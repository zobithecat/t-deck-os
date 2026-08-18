// sanoTTS W8A12 decoder — on-device speed measurement, not a feature.
//
// The question this answers, and the only one worth answering before any of the port
// work: how long does this hardware take to synthesize one second of speech? Every
// published figure for this model is from a desktop — RTF 3.72 on a Ryzen 5 2600, 0.77
// on the Mac that built this — and a 240 MHz Xtensa core is a different animal. If the
// answer is under 1 the decoder can replace eSpeak; if it is 10 it cannot, and no amount
// of I2S plumbing changes that. So it gets measured before anything is rewritten.
//
// Deliberately isolated: its own translation unit, built only under -DTTS_BENCH, and it
// touches neither the audio engine nor the speech queue. eSpeak keeps working either way.
//
// Source: zobithecat/korean-sanotts-esp32s3 @ 07bf690, decoder component verbatim.
//
// MEASURED, T-Deck ESP32-S3 @ 240 MHz, 2026-08-18:
//   24 frames -> 6144 samples = 0.279 s of audio in 120,029 ms
//   RTF 430.8 — one second of speech costs seven minutes of CPU
//   the 5.584 s demo sentence would take 40 minutes
//   workspace 3,235,952 B in PSRAM, weights 1,661,152 B staged to PSRAM
//
// For scale: RTF 0.77 on the Mac that built this, 3.72 on the repo's Ryzen 5 2600. The
// board is 116x the Ryzen figure. Xtensa PIE kernels plus both cores plus internal-SRAM
// weight staging is worth maybe 25x together, which lands at RTF ~17 — and that is for
// the decoder alone, with the duration/acoustic front half (831k more parameters) and
// Korean G2P still to come. Two orders of magnitude is not a kernel problem.
//
// Conclusion: this cannot replace eSpeak-NG here. Kept as the measurement, not a path.
#ifdef TTS_BENCH

#include <Arduino.h>
#include <esp_heap_caps.h>
extern "C" {
#include "korean_tts_decoder.h"
}

extern "C" const uint8_t snt_meta_start[],    snt_meta_end[];
extern "C" const uint8_t snt_weights_start[], snt_weights_end[];
extern "C" const uint8_t snt_latent_start[],  snt_latent_end[];

// 24 frames, not the full 481. One frame is 256 samples at 22,050 Hz, so this is 0.279 s
// of audio — enough to time honestly and short enough that a bad result comes back in
// seconds instead of minutes. Three chunks, so chunk+halo overhead is in the number.
#define BENCH_FRAMES 24

static long g_bench_samples = 0;
static int bench_sink(const float *pcm, int samples, void *user)
{
    (void)pcm; (void)user;
    g_bench_samples += samples;
    return 0;
}

void tts_bench_run()
{
    const size_t meta_n = (size_t)(snt_meta_end - snt_meta_start);
    const size_t wts_n  = (size_t)(snt_weights_end - snt_weights_start);
    const size_t lat_n  = (size_t)(snt_latent_end - snt_latent_start);
    Serial.printf("\n[tts] sanoTTS decoder bench — meta %u B, weights %u B, latent %u B\n",
                  (unsigned)meta_n, (unsigned)wts_n, (unsigned)lat_n);

    // Weights staged into PSRAM rather than read through flash XIP. A real port has to
    // do this anyway (upstream warns vector loads straight off XIP can read wrong
    // values), and leaving them in flash would time the cache, not the arithmetic.
    int8_t *weights = (int8_t *)heap_caps_malloc(wts_n, MALLOC_CAP_SPIRAM);
    if (!weights) { Serial.println("[tts] weights alloc failed"); return; }
    memcpy(weights, snt_weights_start, wts_n);

    korean_tts_decoder_t dec;
    int st = korean_tts_decoder_init_from_buffers(&dec, snt_meta_start, meta_n,
                                                  weights, wts_n,
                                                  (const float *)snt_latent_start, lat_n);
    if (st != 0) { Serial.printf("[tts] init failed: %d\n", st); free(weights); return; }

    size_t ws_n = korean_tts_decoder_workspace_bytes(&dec, KOREAN_TTS_DECODER_CHUNK_FRAMES,
                                                     KOREAN_TTS_DECODER_HALO_FRAMES);
    void  *ws   = heap_caps_malloc(ws_n, MALLOC_CAP_SPIRAM);
    Serial.printf("[tts] workspace %u B  (PSRAM free %u B)\n", (unsigned)ws_n,
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    if (!ws) { Serial.println("[tts] workspace alloc failed"); free(weights); return; }

    g_bench_samples = 0;
    uint32_t t0 = millis();
    st = korean_tts_decoder_synthesize(&dec, (const float *)snt_latent_start, BENCH_FRAMES,
                                       KOREAN_TTS_DECODER_CHUNK_FRAMES,
                                       KOREAN_TTS_DECODER_HALO_FRAMES,
                                       ws, ws_n, bench_sink, NULL);
    uint32_t ms = millis() - t0;

    if (st != 0) { Serial.printf("[tts] synthesize failed: %d\n", st); }
    else {
        double audio_s = (double)g_bench_samples / 22050.0;
        Serial.printf("[tts] %d frames -> %ld samples = %.3f s audio in %lu ms\n",
                      BENCH_FRAMES, g_bench_samples, audio_s, (unsigned long)ms);
        Serial.printf("[tts] RTF %.1f   (1 s of speech costs %.1f s of CPU)\n",
                      (ms / 1000.0) / audio_s, (ms / 1000.0) / audio_s);
        Serial.printf("[tts] the fixed 5.584 s sample would take %.0f s\n",
                      5.584 * (ms / 1000.0) / audio_s);
    }
    free(ws);
    free(weights);
}

#endif /* TTS_BENCH */
