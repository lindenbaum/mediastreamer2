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
 * This is a re-implementation of the audiomixer filter provided in
 * mediastreamer2/src/audiofilters/audiofilter.c using AVX2 instructions.
 */


#include "mediastreamer2/msaudiomixer.h"
#include "mediastreamer2/msticker.h"

#define MIXER_MAX_CHANNELS 128
#define BYPASS_MODE_TIMEOUT 1000

#pragma GCC target ("avx2")

#define VLENGTH 8
#define VHISIZE 16
typedef short v8hi __attribute__((vector_size(VHISIZE)));
#define VSISIZE 32
typedef int v8si __attribute__((vector_size(VSISIZE)));

static void accumulate(v8si *sum, v8hi *contrib, int elems) {
  int i;
  for (i = 0; i < elems; ++i) {
    sum[i] = sum[i] + __builtin_ia32_pmovsxwd256(contrib[i]);
  }
}

static MS2_INLINE short saturate(int s) {
  if (s>32767) return 32767;
  if (s<-32767) return -32767;
  return (short) s;
}

typedef struct Channel {
  MSBufferizer bufferizer;
  v8hi *input; /*the channel contribution, for removal at output*/
  int min_fullness;
  uint64_t last_flow_control;
  uint64_t last_activity;
  bool_t active;
  bool_t output_enabled;
} Channel;

static void channel_init(Channel *chan) {
  ms_bufferizer_init(&chan->bufferizer);
  chan->input = NULL;
  chan->active = TRUE;
  chan->output_enabled = TRUE;
}

static void channel_prepare(Channel *chan, int bytes_per_tick) {
  posix_memalign((void **) &chan->input, VHISIZE, bytes_per_tick);
  chan->last_flow_control = (uint64_t) -1;
  chan->last_activity = (uint64_t) -1;
}

static int channel_process_in(Channel *chan, MSQueue *q, v8si *sum, int nsamples) {
  ms_bufferizer_put_from_queue(&chan->bufferizer, q);
  if (ms_bufferizer_read(&chan->bufferizer, (uint8_t *) chan->input, nsamples*2) != 0) {
    if (chan->active) {
      accumulate(sum, chan->input, nsamples/VLENGTH);
    }
    return nsamples;
  }
  else {
    memset(chan->input, 0, nsamples*2);
  }
  return 0;
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
  if (time-chan->last_flow_control >= 5000) {
    if (chan->min_fullness >= threshold) {
      skip = chan->min_fullness - (threshold/2);
      ms_bufferizer_skip_bytes(&chan->bufferizer, skip);
    }
    chan->last_flow_control = time;
    chan->min_fullness = -1;
  }
  return skip;
}

static mblk_t *channel_process_out(Channel *chan, v8si *sum, int nsamples) {
  int i, j;
  mblk_t *om = allocb(nsamples*2, 0);
  short *out = (short *) om->b_wptr;

  if (chan->active) {
    /*remove own contribution from sum*/
    for (i = 0; i < nsamples/VLENGTH; ++i) {
      v8si o = sum[i] - __builtin_ia32_pmovsxwd256(chan->input[i]);
      for (j = 0; j < VLENGTH; ++j) {
        out[i*VLENGTH + j] = saturate(o[j]);
      }
    }
  }
  else {
    for (i = 0; i < nsamples/VLENGTH; ++i) {
      for (j = 0; j < VLENGTH; ++j) {
        out[i*VLENGTH + j] = saturate(sum[i][j]);
      }
    }
  }
  om->b_wptr += nsamples*2;
  return om;
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
  int bytespertick;
  Channel channels[MIXER_MAX_CHANNELS];
  v8si *sum;
  int conf_mode;
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
  for(i = 0; i < MIXER_MAX_CHANNELS; ++i){
    channel_init(&s->channels[i]);
  }
  f->data = s;
}

static void mixer_uninit(MSFilter *f) {
  int i;
  MixerState *s = (MixerState *) f->data;
  for(i = 0; i < MIXER_MAX_CHANNELS; ++i) {
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

  s->bytespertick = (2*s->nchannels*s->rate*f->ticker->interval)/1000;
  posix_memalign((void **) &s->sum, VSISIZE, (s->bytespertick/2)*sizeof(int));
  for(i = 0; i < MIXER_MAX_CHANNELS; ++i)
    channel_prepare(&s->channels[i], s->bytespertick);
  s->skip_threshold = s->bytespertick*2;
  s->bypass_mode = FALSE;
  s->single_output = has_single_output(f, s);
}

static void mixer_postprocess(MSFilter *f) {
  MixerState *s = (MixerState *) f->data;
  int i;

  free(s->sum);
  s->sum = NULL;
  for(i = 0; i < MIXER_MAX_CHANNELS; ++i)
    channel_unprepare(&s->channels[i]);
}

static mblk_t *make_output(v8si *sum, int nwords) {
  mblk_t *om = allocb(nwords*2, 0);
  short *out = (short *) om->b_wptr;
  int i, j;
  for(i = 0; i < nwords/VLENGTH; ++i) {
    for (j = 0; j < VLENGTH; ++j) {
      out[i*VLENGTH + j] = saturate(sum[i][j]);
    }
  }
  om->b_wptr += nwords*2;
  return om;
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
        for(m = ms_queue_peek_first(inq); !ms_queue_end(inq, m); m = ms_queue_next(inq, m)) {
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
        else if (curtime-chan->last_activity < BYPASS_MODE_TIMEOUT) {
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
  int i;
  int nwords = s->bytespertick/2;

  ms_filter_lock(f);
  if (mixer_check_bypass(f, s)) {
    ms_filter_unlock(f);
    return;
  }

  memset(s->sum, 0, nwords*sizeof(int));

  /* read from all inputs and sum everybody */
  for(i = 0; i < f->desc->ninputs; ++i) {
    MSQueue *q = f->inputs[i];
    if (q) {
      channel_process_in(&s->channels[i], q, s->sum, nwords);
      channel_flow_control(&s->channels[i], s->skip_threshold, f->ticker->time);
    }
  }

  if (s->conf_mode == 0) {
    mblk_t *om = NULL;
    for(i = 0; i < MIXER_MAX_CHANNELS; ++i) {
      MSQueue *q = f->outputs[i];
      Channel *chan = &s->channels[i];
      if (q && chan->output_enabled) {
        if (om == NULL) {
          om = make_output(s->sum, nwords);
        }
        else {
          om = dupb(om);
        }
        ms_queue_put(q, om);
      }
    }
  }
  else {
    for(i = 0; i < MIXER_MAX_CHANNELS; ++i) {
      MSQueue *q = f->outputs[i];
      Channel *chan = &s->channels[i];
      if (q && chan->output_enabled) {
        ms_queue_put(q, channel_process_out(&s->channels[i], s->sum, nwords));
      }
    }
  }
  ms_filter_unlock(f);
}

static int mixer_set_rate(MSFilter *f, void *data) {
  int rate = *(int*) data;
  if (rate == 8000 || rate == 16000) {
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
