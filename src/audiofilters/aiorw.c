/**
 * Copyright (C) 2022 VIER GmbH
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * This is a re-implementation of the API provided in
 * mediastreamer2/src/audiofilters/asyncrw.* using POSIX AIO.
 */

//------------------------------------------------------------------------------
// Include section
//------------------------------------------------------------------------------

#include <aio.h>
#include <errno.h>
#include <pthread.h>
#include <stdint.h>

#include "mediastreamer2/msasync.h"
#include "asyncrw.h"
#include "mediastreamer2/msqueue.h"

//------------------------------------------------------------------------------
// Define section
//------------------------------------------------------------------------------

#define BLOCK_SIZE 4096

//------------------------------------------------------------------------------
// Type section
//------------------------------------------------------------------------------

struct _MSAsyncReader {
  off_t offset;
  bool_t eof; // file read completely?
  bool_t processed; // has aiocb already completed?
  MSBufferizer buf;
  struct aiocb aiocb;
};

typedef enum {
  AIO_WRITER_STATE_IDLE,
  // aiocb has not been submitted to aio_write successfully
  AIO_WRITER_STATE_QUEUED,
  // aiocb is currently writing
  AIO_WRITER_STATE_PROCESSING
} AIO_WRITER_STATE;

struct _MSAsyncWriter {
  off_t offset;
  AIO_WRITER_STATE state;
  MSBufferizer buf;
  struct aiocb aiocb;
};

//------------------------------------------------------------------------------
// Implementation
//------------------------------------------------------------------------------

MSAsyncReader *ms_async_reader_new(int fd, off_t offset) {
  MSAsyncReader *obj = calloc(1, sizeof(*obj));
  if (obj) {
    int error;

    obj->eof = FALSE;
    obj->processed = FALSE;
    obj->offset = offset;
    obj->aiocb.aio_buf = calloc(BLOCK_SIZE, sizeof(uint8_t));
    if (!obj->aiocb.aio_buf) {
      free(obj);
      return NULL;
    }

    ms_bufferizer_init(&obj->buf);

    obj->aiocb.aio_fildes = fd;
    obj->aiocb.aio_offset = obj->offset;
    obj->aiocb.aio_nbytes = BLOCK_SIZE;
    obj->aiocb.aio_sigevent.sigev_notify = SIGEV_NONE;
    obj->aiocb.aio_lio_opcode = LIO_NOP;

    error = aio_read(&obj->aiocb);
    if (error < 0) {
      obj->processed = TRUE;
      if (errno != EAGAIN) {
        ms_error("ms_async_reader_new.aio_read(): %s", strerror(errno));
        ms_bufferizer_flush(&obj->buf);
        free((void *) obj->aiocb.aio_buf);
        free(obj);
        return NULL;
      }
      else {
        ms_error("ms_async_reader_new.aio_read(): EAGAIN");
      }
    }
  }
  return obj;
}

void ms_async_reader_destroy(MSAsyncReader *obj) {
  if (obj) {
    int error = aio_cancel(obj->aiocb.aio_fildes, &obj->aiocb);
    if (error == AIO_NOTCANCELED) {
      const struct aiocb *list[] = {&obj->aiocb};
      aio_suspend(list, 1, NULL);
    }

    if (!obj->processed) {
      error = aio_error(&obj->aiocb);
      if (error == 0) {
        obj->processed = TRUE;
        if (aio_return(&obj->aiocb) < 0) {
          ms_error("ms_async_reader_destroy.aio_return(): %s", strerror(errno));
        }
      }
      else if (error != ECANCELED && error != EINVAL) {
        ms_error("ms_async_reader_destroy.aio_error(): %s", strerror(error));
      }
    }

    ms_bufferizer_flush(&obj->buf);
    free((void *) obj->aiocb.aio_buf);
    free(obj);
  }
}

int ms_async_reader_read(MSAsyncReader *obj, uint8_t *buf, size_t size) {
  if (obj) {
    size_t avail;

    int error = EINVAL;
    if (!obj->processed) {
      error = aio_error(&obj->aiocb);
      if (error == 0) {
        ssize_t read = aio_return(&obj->aiocb);
        if (read > 0) {
          mblk_t *m = allocb(read, 0);
          appendb(m, (const char *) obj->aiocb.aio_buf, (int) read, FALSE);
          ms_bufferizer_put(&obj->buf, m);
          obj->offset += read;
        }
        if (read == 0) {
          obj->eof = TRUE;
        }
        if (read < 0) {
          error = errno;
        }
        obj->processed = TRUE;
      }
    }

    // ECANCELED was caused by seek
    if (error == ECANCELED) {
      ms_bufferizer_flush(&obj->buf);
      error = 0;
    }

    // EINVAL means there was no aio operation
    if (error == EINVAL) {
      error = 0;
    }

    avail = ms_bufferizer_get_avail(&obj->buf);
    if (avail < MAX(size, BLOCK_SIZE) && !obj->eof) {
      if (error == 0) {
        // no read in progress
        obj->processed = FALSE;
        obj->aiocb.aio_offset = obj->offset;
        error = aio_read(&obj->aiocb);
        if (error < 0) {
          error = errno;
          if (error != EAGAIN) {
            ms_error("ms_async_reader_read.aio_read(): %s", strerror(errno));
          }
          else {
            ms_error("ms_async_reader_read.aio_read(): EAGAIN");
          }
        }
      }
    }

    if (avail >= size || obj->eof) {
      return (int) ms_bufferizer_read(&obj->buf, buf, MIN(size, avail));
    }
    else if (error == EINPROGRESS || error == EAGAIN) {
      return -BCTBX_EWOULDBLOCK;
    }
    else {
      return error * -1;
    }
  }

  return -EINVAL;
}

void ms_async_reader_seek(MSAsyncReader *obj, off_t offset) {
  if (obj) {
    obj->eof = FALSE;
    obj->offset = offset;
    int error = aio_cancel(obj->aiocb.aio_fildes, &obj->aiocb);
    if (error == AIO_CANCELED || error == AIO_ALLDONE) {
      ms_bufferizer_flush(&obj->buf);
      obj->processed = FALSE;
      obj->aiocb.aio_offset = obj->offset;
      error = aio_read(&obj->aiocb);
      if (error < 0) {
        obj->processed = TRUE;
        if (errno == EAGAIN) {
          ms_error("ms_async_reader_seek.aio_read(): EAGAIN");
        }
        else {
          ms_error("ms_async_reader_seek.aio_read(): %s", strerror(errno));
        }
      }
    }
  }
}

MSAsyncWriter *ms_async_writer_new(int fd, off_t offset) {
  MSAsyncWriter *obj = calloc(1, sizeof(*obj));
  if (obj) {
    obj->offset = offset;
    obj->state = AIO_WRITER_STATE_IDLE;
    obj->aiocb.aio_buf = calloc(BLOCK_SIZE, sizeof(uint8_t));
    if (!obj->aiocb.aio_buf) {
      free(obj);
      return NULL;
    }

    ms_bufferizer_init(&obj->buf);

    obj->aiocb.aio_fildes = fd;
    obj->aiocb.aio_nbytes = 0;
    obj->aiocb.aio_sigevent.sigev_notify = SIGEV_NONE;
    obj->aiocb.aio_lio_opcode = LIO_NOP;
  }
  return obj;
}

void ms_async_writer_destroy(MSAsyncWriter *obj) {
  if (obj) {
    size_t avail = ms_bufferizer_get_avail(&obj->buf);
    while (avail > 0 || obj->state != AIO_WRITER_STATE_IDLE) {
      int error = 0;

      if (obj->state == AIO_WRITER_STATE_IDLE) {
        avail = ms_bufferizer_read(&obj->buf,
                                   (uint8_t *) obj->aiocb.aio_buf,
                                   MIN(BLOCK_SIZE, avail));
        obj->aiocb.aio_nbytes = avail;
        obj->aiocb.aio_offset = obj->offset;
        obj->state = AIO_WRITER_STATE_QUEUED;
      }

      if (obj->state == AIO_WRITER_STATE_QUEUED) {
        error = aio_write(&obj->aiocb);
      }

      if (error == 0) {
        const struct aiocb *list[] = {&obj->aiocb};
        if (aio_suspend(list, 1, NULL) != 0) {
          ms_error("ms_async_writer_destroy.aio_suspend: (%s)", strerror(errno));
        }

        error = aio_error(&obj->aiocb);
        if (error == 0) {
          ssize_t written = aio_return(&obj->aiocb);
          if (written < 0) {
            ms_error("ms_async_writer_destroy.aio_return: (%s)", strerror(errno));
          }
          else if (written != obj->aiocb.aio_nbytes) {
            ms_error("ms_async_writer_destroy.aio_return: (%lu != %lu)",
                     written,
                     obj->aiocb.aio_nbytes);
          }
          if (written > 0) {
            obj->offset += written;
          }
          obj->state = AIO_WRITER_STATE_IDLE;
        }
        else if (error == ECANCELED) {
          ms_error("ms_async_writer_write.aio_error: (ECANCELED)");
          obj->state = AIO_WRITER_STATE_QUEUED;
          ms_usleep(50000);
        }
        else if (error != EINPROGRESS) {
          ms_error("ms_async_writer_write.aio_error: (%s)", strerror(error));
          obj->state = AIO_WRITER_STATE_IDLE;
        }
      }
      else if (errno == EAGAIN) {
        ms_error("ms_async_writer_destroy.aio_write: (EAGAIN)");
        obj->state = AIO_WRITER_STATE_QUEUED;
        ms_usleep(50000);
      }
      else {
        ms_error("ms_async_writer_destroy.aio_write: (%s)", strerror(errno));
        obj->state = AIO_WRITER_STATE_IDLE;
      }

      avail = ms_bufferizer_get_avail(&obj->buf);
    }

    ms_bufferizer_flush(&obj->buf);
    free((void *) obj->aiocb.aio_buf);
    free(obj);
  }
}

int ms_async_writer_write(MSAsyncWriter *obj, mblk_t *m) {
  if (obj) {
    int error = 0;

    ms_bufferizer_put(&obj->buf, m);

    if (obj->state == AIO_WRITER_STATE_QUEUED) {
      error = aio_write(&obj->aiocb);
      if (error == 0) {
        obj->state = AIO_WRITER_STATE_PROCESSING;
      }
      else {
        ms_error("ms_async_writer_write.aio_write: (%s)", strerror(errno));
      }
      return 0;
    }

    if (obj->state == AIO_WRITER_STATE_PROCESSING) {
      error = aio_error(&obj->aiocb);
      if (error == 0) {
        ssize_t written = aio_return(&obj->aiocb);
        if (written < 0) {
          ms_error("ms_async_writer_write.aio_return: (%s)", strerror(errno));
        }
        else if (written != obj->aiocb.aio_nbytes) {
          ms_error("ms_async_writer_write.aio_return: (%lu != %lu)",
                   written,
                   obj->aiocb.aio_nbytes);
        }
        if (written > 0) {
          obj->offset += written;
        }
        obj->state = AIO_WRITER_STATE_IDLE;
      }
      else if (error == ECANCELED) {
        ms_error("ms_async_writer_write.aio_error: (ECANCELED)");
        obj->state = AIO_WRITER_STATE_QUEUED;
      }
      else if (error != EINPROGRESS) {
        ms_error("ms_async_writer_write.aio_error: (%s)", strerror(error));
        obj->state = AIO_WRITER_STATE_IDLE;
      }
    }

    if (obj->state == AIO_WRITER_STATE_IDLE) {
      size_t avail = ms_bufferizer_get_avail(&obj->buf);
      if (avail >= BLOCK_SIZE) {
        avail = ms_bufferizer_read(&obj->buf,
                                   (uint8_t *) obj->aiocb.aio_buf,
                                   BLOCK_SIZE);
        if (avail > 0) {
          obj->aiocb.aio_nbytes = avail;
          obj->aiocb.aio_offset = obj->offset;
          error = aio_write(&obj->aiocb);
          if (error == 0) {
            obj->state = AIO_WRITER_STATE_PROCESSING;
          }
          else if (errno == EAGAIN) {
            ms_error("ms_async_writer_write.aio_write: (EAGAIN)");
            obj->state = AIO_WRITER_STATE_QUEUED;
          }
          else {
            ms_error("ms_async_writer_write.aio_write: (%s)", strerror(errno));
          }
        }
      }
    }

    return 0;
  }

  freemsg(m);
  return -1;
}
