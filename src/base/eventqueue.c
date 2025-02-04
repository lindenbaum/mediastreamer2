/*
 * Copyright (c) 2010-2019 Belledonne Communications SARL.
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
 */


#include "mediastreamer2/mseventqueue.h"
#include "mediastreamer2/msfilter.h"

#ifndef MS_EVENT_QUEUE_MAX_SIZE
#define MS_EVENT_QUEUE_MAX_SIZE 1024
#endif

typedef enum {
	OnlySynchronous,
	OnlyAsynchronous,
	Both
}InvocationMode;

static void ms_filter_invoke_callbacks(MSFilter **f, unsigned int id, void *arg, InvocationMode synchronous_mode);

struct _MSNotifyContext{
	MSFilterNotifyFunc fn;
	void *ud;
	int synchronous;
};

typedef struct _MSNotifyContext MSNotifyContext;

struct _MSEventQueue{
	ms_mutex_t mutex; /*could be replaced by an atomic counter for freeroom*/
	MSFilter *current_notifier;
	queue_t q;
};

typedef struct {
	MSFilter* filter;
	unsigned int ev_id;
	int pad; /* So that next is 64 bit aligned*/
} MSEventHeader;


static void write_event(MSEventQueue *q, MSFilter *f, unsigned int ev_id, void *arg){
	int argsize=ev_id & 0xff;
	int header_size = sizeof(MSEventHeader);
	mblk_t *event_message;

	if (q->q.q_mcount > MS_EVENT_QUEUE_MAX_SIZE){
		ms_error("Mediastreamer2 event queue is stalled, discarding event.");
		return;
	}

	event_message = allocb(header_size + argsize, 0);

	((MSEventHeader *)event_message->b_wptr)->filter = f;
	((MSEventHeader *)event_message->b_wptr)->ev_id = ev_id;

	event_message->b_wptr += header_size;

	if (argsize > 0) {
		memcpy(event_message->b_wptr, arg, argsize);
		event_message->b_wptr += argsize;
	}

	ms_mutex_lock(&q->mutex);
	putq(&q->q, event_message);
	ms_mutex_unlock(&q->mutex);
}

static void parse_event(uint8_t *rptr, MSFilter **f, unsigned int *id, void **data, int *argsize){
	int header_size = sizeof(MSEventHeader);

	if (((intptr_t)rptr % 4) != 0) ms_fatal("Unaligned access");
	*f = ((MSEventHeader *)rptr)->filter;
	*id = ((MSEventHeader *)rptr)->ev_id;

	*argsize = (*id) & 0xff;
	*data = rptr + header_size;
}

static bool_t read_event(MSEventQueue *q){
	mblk_t *event_message;
	bool_t has_read = FALSE;

	ms_mutex_lock(&q->mutex);
	event_message = getq(&q->q);
	ms_mutex_unlock(&q->mutex);
	
	if (event_message){
		MSFilter *f;
		unsigned int id;
		void *data;
		int argsize;

		parse_event(event_message->b_rptr,&f,&id,&data,&argsize);
		if (f) {
			q->current_notifier=f;
			ms_filter_invoke_callbacks(&q->current_notifier,id,argsize>0 ? data : NULL, OnlyAsynchronous);
			q->current_notifier=NULL;
		}
		freeb(event_message);
		has_read = TRUE;
	}
	return has_read;
}

/*clean all events belonging to a MSFilter that is about to be destroyed*/
void ms_event_queue_clean(MSEventQueue *q, MSFilter *destroyed){
	queue_t freeq;
	mblk_t *event_message;
	int cleaned_events = 0;
	
	qinit(&freeq);
	ms_mutex_lock(&q->mutex);
	for (event_message = qbegin(&q->q); !qend(&q->q, event_message); ){
		MSFilter *f;
		unsigned int id;
		void *data;
		int argsize;
		mblk_t *next_event_message = event_message->b_next;

		parse_event(event_message->b_rptr,&f,&id,&data,&argsize);
		if (f == destroyed){
			cleaned_events++;
			remq(&q->q, event_message);
			putq(&freeq, event_message);
		}
		event_message = next_event_message;
	}
	ms_mutex_unlock(&q->mutex);
	if (cleaned_events > 0){
		ms_message("Cleaned [%i] pending event(s) generated by MSFilter [%s:%p]", cleaned_events, destroyed->desc->name, destroyed);
	}
	if (q->current_notifier==destroyed){
		q->current_notifier=NULL;
	}
	flushq(&freeq, 0);
}

MSEventQueue *ms_event_queue_new(){
	MSEventQueue *q=ms_new0(MSEventQueue,1);
	ms_mutex_init(&q->mutex,NULL);
	qinit(&q->q);
	return q;
}

void ms_event_queue_destroy(MSEventQueue *q){
	flushq(&q->q, 0);
	ms_mutex_destroy(&q->mutex);
	ms_free(q);
}

void ms_event_queue_skip(MSEventQueue *q){
	ms_mutex_lock(&q->mutex);
	flushq(&q->q, 0);
	ms_mutex_unlock(&q->mutex);
}


void ms_event_queue_pump(MSEventQueue *q){
	while(read_event(q)){
	}
}

static MSNotifyContext * ms_notify_context_new(MSFilterNotifyFunc fn, void *ud, bool_t synchronous){
	MSNotifyContext *ctx=ms_new0(MSNotifyContext,1);
	ctx->fn=fn;
	ctx->ud=ud;
	ctx->synchronous=synchronous;
	return ctx;
}

static void ms_notify_context_destroy(MSNotifyContext *obj){
	ms_free(obj);
}

void ms_filter_add_notify_callback(MSFilter *f, MSFilterNotifyFunc fn, void *ud, bool_t synchronous){
	f->notify_callbacks=bctbx_list_append(f->notify_callbacks,ms_notify_context_new(fn,ud,synchronous));
}

void ms_filter_remove_notify_callback(MSFilter *f, MSFilterNotifyFunc fn, void *ud){
	bctbx_list_t *elem;
	bctbx_list_t *found=NULL;
	for(elem=f->notify_callbacks;elem!=NULL;elem=elem->next){
		MSNotifyContext *ctx=(MSNotifyContext*)elem->data;
		if (ctx->fn==fn && ctx->ud==ud){
			found=elem;
			break;
		}
	}
	if (found){
		ms_notify_context_destroy((MSNotifyContext*)found->data);
		f->notify_callbacks=bctbx_list_erase_link(f->notify_callbacks,found);
	}else ms_warning("ms_filter_remove_notify_callback(filter=%p): no registered callback with fn=%p and ud=%p",f,fn,ud);
}

void ms_filter_clear_notify_callback(MSFilter *f){
	f->notify_callbacks=bctbx_list_free_with_data(f->notify_callbacks,(void (*)(void*))ms_notify_context_destroy);
}

static void ms_filter_invoke_callbacks(MSFilter **f, unsigned int id, void *arg, InvocationMode synchronous_mode){
	bctbx_list_t *elem;
	for (elem=(*f)->notify_callbacks;elem!=NULL;elem=elem->next){
		MSNotifyContext *ctx=(MSNotifyContext*)elem->data;
		if (synchronous_mode==Both || (synchronous_mode==OnlyAsynchronous && !ctx->synchronous)
			|| (synchronous_mode==OnlySynchronous && ctx->synchronous)){
			ctx->fn(ctx->ud,*f,id,arg);
		}
		if (*f == NULL) break; /*the filter was destroyed by a callback invocation*/
	}
}

void ms_filter_set_notify_callback(MSFilter *f, MSFilterNotifyFunc fn, void *ud){
	ms_filter_add_notify_callback(f,fn,ud,FALSE);
}


void ms_filter_notify(MSFilter *f, unsigned int id, void *arg){
	if (f->notify_callbacks!=NULL){
		if (f->factory->evq==NULL){
			/* synchronous notification */
			ms_filter_invoke_callbacks(&f,id,arg,Both);
		}else{
			ms_filter_invoke_callbacks(&f,id,arg,OnlySynchronous);
			write_event(f->factory->evq,f,id,arg);
		}
	}
}

void ms_filter_notify_no_arg(MSFilter *f, unsigned int id){
	ms_filter_notify(f,id,NULL);
}

void ms_filter_clean_pending_events(MSFilter *f){
	if (f->factory->evq)
		ms_event_queue_clean(f->factory->evq,f);
}

/* we need this pragma because this file implements much of compatibility functions*/
#ifdef _MSC_VER
#pragma warning(disable : 4996)
#else
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

void ms_set_global_event_queue(MSEventQueue *q){
	ms_factory_set_event_queue(ms_factory_get_fallback(),q);
}


