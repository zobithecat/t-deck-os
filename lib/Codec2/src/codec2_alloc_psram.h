// Force-included into every Codec2 TU (library.json: -include codec2_alloc_psram.h).
// All codec2 state goes to PSRAM: this board runs its internal heap at kilobyte
// margins, and the vocoder must not be one more internal-RAM customer. free() is
// heap-agnostic on ESP-IDF, so only the allocators are redirected.
#pragma once
#include <stdlib.h>
#include "esp_heap_caps.h"
static inline void *c2_psram_malloc(size_t n)            { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); }
static inline void *c2_psram_calloc(size_t n, size_t s)  { return heap_caps_calloc(n, s, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); }
#define malloc(n)    c2_psram_malloc(n)
#define calloc(n, s) c2_psram_calloc((n), (s))
