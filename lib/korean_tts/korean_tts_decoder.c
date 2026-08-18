#include "korean_tts_decoder.h"

#include <stdint.h>
#include <string.h>

static uintptr_t align16(uintptr_t value) {
    return (value + 15u) & ~(uintptr_t)15u;
}

int korean_tts_decoder_init_from_buffers(korean_tts_decoder_t *decoder,
                                         const void *meta, size_t meta_bytes,
                                         const int8_t *weights, size_t weight_bytes,
                                         const float *sample_latent, size_t sample_latent_bytes) {
    int rc;
    if (!decoder || !meta || !weights || !sample_latent) return -1;
    memset(decoder, 0, sizeof(*decoder));
    rc = snt_piperlite_q8_init(&decoder->model, meta, meta_bytes, weights, weight_bytes);
    if (rc) return rc;
    if (sample_latent_bytes == 0 ||
        sample_latent_bytes % ((size_t)decoder->model.in_ch * sizeof(float))) return -20;
    decoder->sample_latent = sample_latent;
    decoder->sample_frames = (int)(sample_latent_bytes /
                                   ((size_t)decoder->model.in_ch * sizeof(float)));
    return 0;
}

size_t korean_tts_decoder_workspace_bytes(const korean_tts_decoder_t *decoder,
                                          int chunk_frames, int halo_frames) {
    int window;
    size_t latent_bytes;
    size_t audio_bytes;
    size_t decoder_bytes;
    if (!decoder || chunk_frames <= 0 || halo_frames < 0) return 0;
    window = chunk_frames + 2 * halo_frames;
    latent_bytes = (size_t)decoder->model.in_ch * (size_t)window * sizeof(float);
    audio_bytes = (size_t)window * KOREAN_TTS_DECODER_HOP * sizeof(float);
    decoder_bytes = snt_piperlite_q8_arena_bytes(&decoder->model, window);
    return 48 + latent_bytes + audio_bytes + decoder_bytes;
}

int korean_tts_decoder_synthesize(const korean_tts_decoder_t *decoder,
                                  const float *latent, int frames,
                                  int chunk_frames, int halo_frames,
                                  void *workspace, size_t workspace_bytes,
                                  korean_tts_pcm_sink_t sink, void *user) {
    uintptr_t cursor;
    float *window_latent;
    float *window_audio;
    void *arena;
    size_t max_window;
    size_t latent_bytes;
    size_t audio_bytes;
    size_t arena_bytes;
    int core_start;
    if (!decoder || !latent || frames <= 0 || chunk_frames <= 0 || halo_frames < 0 || !workspace || !sink)
        return -1;
    if (workspace_bytes < korean_tts_decoder_workspace_bytes(decoder, chunk_frames, halo_frames)) return -2;
    max_window = (size_t)(chunk_frames + 2 * halo_frames);
    cursor = align16((uintptr_t)workspace);
    window_latent = (float *)(void *)cursor;
    latent_bytes = (size_t)decoder->model.in_ch * max_window * sizeof(float);
    cursor = align16(cursor + latent_bytes);
    window_audio = (float *)(void *)cursor;
    audio_bytes = max_window * KOREAN_TTS_DECODER_HOP * sizeof(float);
    cursor = align16(cursor + audio_bytes);
    arena = (void *)cursor;
    arena_bytes = workspace_bytes - (size_t)(cursor - (uintptr_t)workspace);

    for (core_start = 0; core_start < frames; core_start += chunk_frames) {
        int core_end = core_start + chunk_frames;
        int input_start;
        int input_end;
        int input_frames;
        int core_frames;
        int channel;
        int rc;
        const float *emit;
        if (core_end > frames) core_end = frames;
        input_start = core_start - halo_frames;
        if (input_start < 0) input_start = 0;
        input_end = core_end + halo_frames;
        if (input_end > frames) input_end = frames;
        input_frames = input_end - input_start;
        core_frames = core_end - core_start;
        for (channel = 0; channel < decoder->model.in_ch; channel++) {
            memcpy(window_latent + (size_t)channel * input_frames,
                   latent + (size_t)channel * frames + input_start,
                   (size_t)input_frames * sizeof(float));
        }
        rc = snt_piperlite_q8_synthesize(&decoder->model, window_latent, input_frames,
                                         window_audio, arena, arena_bytes);
        if (rc) return -10 + rc;
        emit = window_audio + (size_t)(core_start - input_start) * KOREAN_TTS_DECODER_HOP;
        rc = sink(emit, core_frames * KOREAN_TTS_DECODER_HOP, user);
        if (rc) return rc;
    }
    return 0;
}
