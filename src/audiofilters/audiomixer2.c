/*
 * Copyright (C) 2022 VIER GmbH
 *
 * This file is part of mediastreamer2.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * An optimized re-implementation of the audiomixer filter provided in
 * mediastreamer2/src/audiofilters/audiofilter.c. It uses constructs that are
 * (hopefully) recognized by the GCC auto-vectorizer. It comes for the prize of
 * reduced features.
 */

#include "mediastreamer2/msaudiomixer.h"
#include "mediastreamer2/msticker.h"

#define MIXER_MAX_CHANNELS 128
#define BYPASS_MODE_TIMEOUT 1000

static MS2_INLINE short saturate_sample(int s) {
  return (short) (s > 32767 ? 32767 : (s < -32767 ? -32767 : s));
}

static MS2_INLINE void accumulate(int * __restrict__ sum,
                                  short * __restrict__ input,
                                  int nsamples) {
  int i;
  for (i = 0; i < nsamples; ++i) {
    sum[i] = sum[i] + input[i];
  }
}

static MS2_INLINE void subtract_and_copy_to_out(short * __restrict__ out,
                                                int * __restrict__ sum,
                                                short * __restrict__ input,
                                                int nsamples) {
  int i;
  for (i = 0; i < nsamples; ++i) {
    out[i] = saturate_sample(sum[i] - input[i]);
  }
}

static MS2_INLINE void copy_to_out(short * __restrict__ out,
                                   int * __restrict__ sum,
                                   int nsamples) {
  int i;
  for (i = 0; i < nsamples; ++i) {
    out[i] = saturate_sample(sum[i]);
  }
}

typedef struct Channel {
  MSBufferizer bufferizer;
  short *input;
  int min_fullness;
  uint64_t last_flow_control;
  uint64_t last_activity;
  bool_t active;
  bool_t output_enabled;
  bool_t had_input;
} Channel;

static void channel_init(Channel *chan) {
  ms_bufferizer_init(&chan->bufferizer);
  chan->input = NULL;
  chan->active = TRUE;
  chan->output_enabled = TRUE;
  chan->had_input = FALSE;
}

static void channel_prepare(Channel *chan, int samples_per_tick) {
  posix_memalign((void **) &chan->input, 16, samples_per_tick*sizeof(short));
  chan->last_flow_control = (uint64_t) -1;
  chan->last_activity = (uint64_t) -1;
}

static int channel_flow_control(Channel *chan, int threshold, uint64_t time) {
  int size;
  int skip = 0;
  if (chan->last_flow_control == (uint64_t) -1) {
    chan->last_flow_control = time;
    chan->min_fullness = -1;
    return skip;
  }
  size = (int) ms_bufferizer_get_avail(&chan->bufferizer);
  if (chan->min_fullness == -1 || size < chan->min_fullness)
    chan->min_fullness = size;
  if (time - chan->last_flow_control >= 5000) {
    if (chan->min_fullness >= threshold) {
      skip = chan->min_fullness - (threshold/2);
      ms_bufferizer_skip_bytes(&chan->bufferizer, skip);
    }
    chan->last_flow_control = time;
    chan->min_fullness = -1;
  }
  return skip;
}

static void channel_unprepare(Channel *chan) {
  free(chan->input);
  chan->input=NULL;
}

static void channel_uninit(Channel *chan) {
  ms_bufferizer_uninit(&chan->bufferizer);
}

typedef struct MixerState {
  int nchannels;
  int rate;
  int samplespertick;
  Channel channels[MIXER_MAX_CHANNELS];
  int conf_mode;
  int *sum;
  int skip_threshold;
  bool_t bypass_mode;
  bool_t single_output;
} MixerState;

static void mixer_init(MSFilter *f) {
  MixerState *s = ms_new0(MixerState, 1);
  int i;
  s->conf_mode = FALSE; /*this is the default, don't change it*/
  s->nchannels = 1;
  s->rate = 16000;
  for (i = 0; i < MIXER_MAX_CHANNELS; ++i){
    channel_init(&s->channels[i]);
  }
  f->data = s;
}

static void mixer_uninit(MSFilter *f) {
  int i;
  MixerState *s = (MixerState *) f->data;
  for (i = 0; i < MIXER_MAX_CHANNELS; ++i) {
    channel_uninit(&s->channels[i]);
  }
  ms_free(s);
}

static bool_t has_single_output(MSFilter *f, MixerState *s) {
  int i;
  int count = 0;
  for (i = 0; i < f->desc->noutputs; ++i) {
    Channel *chan = &s->channels[i];
    if (f->outputs[i] && chan->output_enabled) count++;
  }
  return count == 1;
}

static void mixer_preprocess(MSFilter *f) {
  MixerState *s = (MixerState *) f->data;
  int i;

  s->samplespertick = (s->nchannels*s->rate*f->ticker->interval)/1000;
  posix_memalign((void **) &s->sum, 32, s->samplespertick*sizeof(int));
  for (i = 0; i < MIXER_MAX_CHANNELS; ++i)
    channel_prepare(&s->channels[i], s->samplespertick);
  s->skip_threshold = s->samplespertick*4;
  s->bypass_mode = FALSE;
  s->single_output = has_single_output(f, s);
}

static void mixer_postprocess(MSFilter *f) {
  MixerState *s = (MixerState *) f->data;
  int i;

  free(s->sum);
  s->sum = NULL;
  for (i = 0; i < MIXER_MAX_CHANNELS; ++i)
    channel_unprepare(&s->channels[i]);
}

static void mixer_dispatch_output(MSFilter *f, MixerState*s, MSQueue *inq, int active_input) {
  int i;
  for (i = 0; i < f->desc->noutputs; i++) {
    MSQueue *outq = f->outputs[i];
    Channel *chan = &s->channels[i];
    if (outq && chan->output_enabled && (active_input != i || s->conf_mode == 0)) {
      mblk_t *m;
      if (s->single_output) {
        while((m = ms_queue_get(inq)) != NULL) {
          ms_queue_put(outq, m);
        }
        break;
      }
      else {
        for (m = ms_queue_peek_first(inq); !ms_queue_end(inq, m); m = ms_queue_next(inq, m)) {
          ms_queue_put(outq, dupmsg(m));
        }
      }
    }
  }
  ms_queue_flush(inq);
}

/* the bypass mode is an optimization for the case of a single contributing channel. In such case there is no need to synchronize with other channels
 * and to make a sum. The processing is greatly simplified by just distributing the packets from the single contributing channels to the output channels.*/
static bool_t mixer_check_bypass(MSFilter *f, MixerState *s) {
  int i;
  int active_cnt = 0;
  int active_input = -1;
  MSQueue *activeq = NULL;
  uint64_t curtime = f->ticker->time;
  for (i = 0; i < f->desc->ninputs; i++) {
    MSQueue *q = f->inputs[i];
    if (q) {
      Channel *chan = &s->channels[i];
      if (!ms_queue_empty(q)) {
        chan->last_activity = curtime;
        activeq = q;
        active_cnt++;
        active_input = i;
      }
      else {
        if (chan->last_activity == (uint64_t) -1) {
          chan->last_activity = curtime;
        }
        else if (curtime - chan->last_activity < BYPASS_MODE_TIMEOUT) {
          activeq = q;
          active_cnt++;
          active_input = i;
        }
      }
    }
  }
  if (active_cnt == 1) {
    if (!s->bypass_mode) {
      s->bypass_mode = TRUE;
      ms_message("MSAudioMixer [%p] is entering bypass mode.", f);
    }
    mixer_dispatch_output(f, s, activeq, active_input);
    return TRUE;
  }
  else if (active_cnt > 1) {
    if (s->bypass_mode) {
      s->bypass_mode = FALSE;
      ms_message("MSAudioMixer [%p] is leaving bypass mode.", f);
    }
    return FALSE;
  }
  /*last case: no contributing channels at all. There is then nothing to do.*/
  return TRUE;
}

static void mixer_process(MSFilter *f) {
  MixerState *s = (MixerState *) f->data;
  mblk_t *om_cached = NULL;
  int i, bytespertick = s->samplespertick*sizeof(short);

  ms_filter_lock(f);
  if (mixer_check_bypass(f, s)) {
    ms_filter_unlock(f);
    return;
  }

  memset(s->sum, 0, s->samplespertick*sizeof(int));

  /* read from all inputs and sum everybody */
  for (i = 0; i < f->desc->ninputs; ++i) {
    MSQueue *q = f->inputs[i];
    if (q) {
      Channel *chan = &s->channels[i];
      chan->had_input = FALSE;

      ms_bufferizer_put_from_queue(&chan->bufferizer, q);
      if (ms_bufferizer_read(&chan->bufferizer, (uint8_t *) chan->input, bytespertick) != 0) {
        if (chan->active) {
          accumulate(s->sum, chan->input, s->samplespertick);
          chan->had_input = TRUE;
        }
      }
      channel_flow_control(chan, s->skip_threshold, f->ticker->time);
    }
  }

  for (i = 0; i < MIXER_MAX_CHANNELS; ++i) {
    MSQueue *q = f->outputs[i];
    Channel *chan = &s->channels[i];
    if (q && chan->output_enabled) {
      mblk_t *om = NULL;
      if (chan->active && chan->had_input && s->conf_mode) {
        om = allocb(bytespertick, 0);
        subtract_and_copy_to_out((short *) om->b_wptr, s->sum, chan->input, s->samplespertick);
        om->b_wptr += bytespertick;
      }
      else if (om_cached == NULL) {
        om = allocb(bytespertick, 0);
        copy_to_out((short *) om->b_wptr, s->sum, s->samplespertick);
        om->b_wptr += bytespertick;
        om_cached = om;
      }
      else {
        om = dupb(om_cached);
      }
      ms_queue_put(q, om);
    }
  }
  ms_filter_unlock(f);
}

static int mixer_set_rate(MSFilter *f, void *data) {
  int rate = *(int*) data;
  if (rate % 8000 == 0) {
    MixerState *s = (MixerState *) f->data;
    s->rate = rate;
    return 0;
  }
  ms_warning("mixer_set_rate: unsupported sampling rate %i", rate);
  return -1;
}

static int mixer_get_rate(MSFilter *f, void *data) {
  MixerState *s = (MixerState *) f->data;
  *(int*) data = s->rate;
  return 0;
}

static int mixer_set_nchannels(MSFilter *f, void *data) {
  MixerState *s = (MixerState *) f->data;
  s->nchannels = *(int*) data;
  return 0;
}

static int mixer_get_nchannels(MSFilter *f, void *data) {
  MixerState *s = (MixerState *) f->data;
  *(int*) data = s->nchannels;
  return 0;
}

static int mixer_set_input_gain(MSFilter *f, void *data) {
  (void) f;
  (void) data;
  ms_warning("mixer_set_input_gain: not implemented");
  return -1;
}

static int mixer_set_active(MSFilter *f, void *data) {
  MixerState *s = (MixerState *) f->data;
  MSAudioMixerCtl *ctl = (MSAudioMixerCtl*) data;
  if (ctl->pin < 0 || ctl->pin >= MIXER_MAX_CHANNELS) {
    ms_warning("mixer_set_active: invalid pin number %i", ctl->pin);
    return -1;
  }
  s->channels[ctl->pin].active = ctl->param.active;
  return 0;
}

static int mixer_enable_output(MSFilter *f, void *data) {
  MixerState *s = (MixerState *) f->data;
  MSAudioMixerCtl *ctl = (MSAudioMixerCtl*) data;
  if (ctl->pin < 0 || ctl->pin >= MIXER_MAX_CHANNELS) {
    ms_warning("mixer_enable_output: invalid pin number %i", ctl->pin);
    return -1;
  }
  ms_filter_lock(f);
  s->channels[ctl->pin].output_enabled = ctl->param.enabled;
  s->single_output=has_single_output(f, s);
  ms_filter_unlock(f);
  return 0;
}

static int mixer_set_conference_mode(MSFilter *f, void *data) {
  MixerState *s = (MixerState *) f->data;
  s->conf_mode = *(int*) data;
  return 0;
}

static int mixer_set_master_channel(MSFilter *f, void *data) {
  (void) f;
  (void) data;
  return 0;
}

static MSFilterMethod methods[] = {
  {MS_FILTER_SET_NCHANNELS, mixer_set_nchannels},
  {MS_FILTER_GET_NCHANNELS, mixer_get_nchannels},
  {MS_FILTER_SET_SAMPLE_RATE, mixer_set_rate},
  {MS_FILTER_GET_SAMPLE_RATE, mixer_get_rate},
  {MS_AUDIO_MIXER_SET_INPUT_GAIN, mixer_set_input_gain},
  {MS_AUDIO_MIXER_SET_ACTIVE, mixer_set_active},
  {MS_AUDIO_MIXER_ENABLE_CONFERENCE_MODE, mixer_set_conference_mode},
  {MS_AUDIO_MIXER_SET_MASTER_CHANNEL, mixer_set_master_channel},
  {MS_AUDIO_MIXER_ENABLE_OUTPUT, mixer_enable_output},
  {0, NULL}
};

MSFilterDesc ms_audio_mixer_desc = {
  .id = MS_AUDIO_MIXER_ID,
  .name = "MSAudioMixer",
  .text = N_("A filter that mixes down 16 bit sample audio streams"),
  .category = MS_FILTER_OTHER,
  .ninputs = MIXER_MAX_CHANNELS,
  .noutputs = MIXER_MAX_CHANNELS,
  .init = mixer_init,
  .preprocess = mixer_preprocess,
  .process = mixer_process,
  .postprocess = mixer_postprocess,
  .uninit = mixer_uninit,
  .methods = methods,
  .flags = MS_FILTER_IS_PUMP
};

MS_FILTER_DESC_EXPORT(ms_audio_mixer_desc)
