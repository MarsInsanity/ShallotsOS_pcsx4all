#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "tf_audio_queue.h"

int main(void)
{
    tf_audio_queue q;
    tf_audio_frame out[TF_AUDIO_CHUNK];
    int16_t input[1000 * 2];
    for (unsigned i = 0; i < 1000 * 2; i++) input[i] = 10000;

    assert(tf_audio_queue_init(&q, 48000, 10) == 0);
    tf_audio_queue_push(&q, input, 400);
    assert(!tf_audio_queue_pull(&q, out)); /* prefill */
    tf_audio_queue_push(&q, input, 600);
    assert(tf_audio_queue_pull(&q, out));
    assert(tf_audio_queue_pull(&q, out));
    assert(tf_audio_queue_pull(&q, out)); /* short batch fades into silence */
    assert(out[TF_AUDIO_CHUNK - 1].left == 0);
    tf_audio_queue_push(&q, input, 480);
    assert(tf_audio_queue_pull(&q, out));
    assert(out[0].left > 0 && out[0].left < 1000);
    assert(out[239].left == 10000);
    unsigned underruns = 0, overruns = 0;
    tf_audio_queue_counts(&q, &underruns, &overruns);
    assert(underruns == 1 && overruns == 0);
    tf_audio_queue_destroy(&q);

    assert(tf_audio_queue_init(&q, 44100, 10) == 0);
    tf_audio_queue_push(&q, input, 441);
    assert(tf_audio_queue_pull(&q, out)); /* 44.1 -> 48 kHz supplies 10 ms */
    tf_audio_queue_destroy(&q);

    for (unsigned i = 0; i < 1000 * 2; i++)
        input[i] = i & 2 ? INT16_MIN : INT16_MAX;
    assert(tf_audio_queue_init(&q, 44100, 10) == 0);
    tf_audio_queue_push(&q, input, 1000); /* full-scale interpolation */
    assert(tf_audio_queue_pull(&q, out));
    tf_audio_queue_destroy(&q);
    puts("tf_audio_queue: OK");
    return 0;
}
