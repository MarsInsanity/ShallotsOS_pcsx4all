#include "tf_audio_queue.h"

#include <string.h>

int tf_audio_queue_init(tf_audio_queue *q, int input_rate, int target_ms)
{
    if (!q || input_rate <= 0 || target_ms < 10 || target_ms > 150)
        return -1;
    memset(q, 0, sizeof(*q));
    q->input_rate = input_rate;
    q->target = target_ms * 48;
    return pthread_mutex_init(&q->lock, NULL);
}

void tf_audio_queue_destroy(tf_audio_queue *q)
{
    pthread_mutex_destroy(&q->lock);
}

void tf_audio_queue_push(tf_audio_queue *q, const int16_t *stereo, unsigned frames)
{
    if (!stereo || !frames) return;
    pthread_mutex_lock(&q->lock);
    unsigned queued = q->write - q->read;
    int wanted = queued < (unsigned)q->target
        ? (int)(((uint64_t)(q->target - queued) << 16) / (q->target * 8u)) : 0;
    q->slow_q16 += (wanted - q->slow_q16) / 8;
    uint32_t step = (uint32_t)(((uint64_t)q->input_rate << 16) / 48000u);
    step -= (uint32_t)(((uint64_t)step * q->slow_q16) >> 16);
    if (queued > (unsigned)q->target + TF_AUDIO_CHUNK) step += step >> 9;

    for (unsigned i = 0; i < frames; i++) {
        tf_audio_frame cur = { stereo[i * 2], stereo[i * 2 + 1] };
        while (q->phase < 0x10000) {
            int frac = q->phase;
            tf_audio_frame out = {
                (int16_t)(q->previous.left +
                    ((int64_t)(cur.left - q->previous.left) * frac >> 16)),
                (int16_t)(q->previous.right +
                    ((int64_t)(cur.right - q->previous.right) * frac >> 16))
            };
            if (q->write - q->read >= TF_AUDIO_RING) {
                q->read += TF_AUDIO_CHUNK;
                q->overruns++;
            }
            q->ring[q->write & (TF_AUDIO_RING - 1)] = out;
            q->write++;
            q->phase += step;
        }
        q->phase -= 0x10000;
        q->previous = cur;
    }
    pthread_mutex_unlock(&q->lock);
}

int tf_audio_queue_pull(tf_audio_queue *q, tf_audio_frame out[TF_AUDIO_CHUNK])
{
    pthread_mutex_lock(&q->lock);
    unsigned avail = q->write - q->read;
    if (!q->primed && avail < (unsigned)q->target) {
        pthread_mutex_unlock(&q->lock);
        return 0;
    }
    q->primed = 1;
    unsigned take = avail < TF_AUDIO_CHUNK ? avail : TF_AUDIO_CHUNK;
    for (unsigned i = 0; i < take; i++)
        out[i] = q->ring[q->read++ & (TF_AUDIO_RING - 1)];

    if (q->recovering && take) {
        unsigned fade = take < 240 ? take : 240; /* 5 ms at 48 kHz */
        for (unsigned i = 0; i < fade; i++) {
            out[i].left = (int16_t)(q->last.left +
                ((int32_t)out[i].left - q->last.left) * (i + 1) / fade);
            out[i].right = (int16_t)(q->last.right +
                ((int32_t)out[i].right - q->last.right) * (i + 1) / fade);
        }
        q->recovering = 0;
    }
    if (take < TF_AUDIO_CHUNK) {
        unsigned missing = TF_AUDIO_CHUNK - take;
        tf_audio_frame from = take ? out[take - 1] : q->last;
        for (unsigned i = 0; i < missing; i++) {
            unsigned remain = missing <= 160 ? missing : missing - i - 1;
            out[take + i].left = (int16_t)((int32_t)from.left * remain / missing);
            out[take + i].right = (int16_t)((int32_t)from.right * remain / missing);
        }
        q->underruns++;
        q->recovering = 1;
    }
    q->last = out[TF_AUDIO_CHUNK - 1];
    pthread_mutex_unlock(&q->lock);
    return 1;
}

void tf_audio_queue_counts(tf_audio_queue *q, unsigned *underruns, unsigned *overruns)
{
    pthread_mutex_lock(&q->lock);
    if (underruns) *underruns = q->underruns;
    if (overruns) *overruns = q->overruns;
    pthread_mutex_unlock(&q->lock);
}
