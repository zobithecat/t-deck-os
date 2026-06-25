#pragma once
#include <stdint.h>

// QWERTY lowercase letter + shift → Hangul compatibility jamo (U+3131..U+3163).
// Standard 두벌식 layout. Returns 0 if no mapping.
//
// Layout (qwerty -> jamo / shift-jamo):
//   q ㅂ/ㅃ   w ㅈ/ㅉ   e ㄷ/ㄸ   r ㄱ/ㄲ   t ㅅ/ㅆ
//   y ㅛ      u ㅕ      i ㅑ      o ㅐ/ㅒ   p ㅔ/ㅖ
//   a ㅁ      s ㄴ      d ㅇ      f ㄹ      g ㅎ
//   h ㅗ      j ㅓ      k ㅏ      l ㅣ
//   z ㅋ      x ㅌ      c ㅊ      v ㅍ      b ㅠ
//   n ㅜ      m ㅡ
inline uint16_t dubeolsik_lookup(char qwerty_lower, bool shift) {
  // Indexed by (qwerty_lower - 'a'), two columns: [no-shift, shift]
  // 0 means no mapping.
  static const uint16_t map[26][2] = {
    /* a */ { 0x3141, 0x3141 }, // ㅁ
    /* b */ { 0x3160, 0x3160 }, // ㅠ
    /* c */ { 0x314A, 0x314A }, // ㅊ
    /* d */ { 0x3147, 0x3147 }, // ㅇ
    /* e */ { 0x3137, 0x3138 }, // ㄷ / ㄸ
    /* f */ { 0x3139, 0x3139 }, // ㄹ
    /* g */ { 0x314E, 0x314E }, // ㅎ
    /* h */ { 0x3157, 0x3157 }, // ㅗ
    /* i */ { 0x3151, 0x3151 }, // ㅑ
    /* j */ { 0x3153, 0x3153 }, // ㅓ
    /* k */ { 0x314F, 0x314F }, // ㅏ
    /* l */ { 0x3163, 0x3163 }, // ㅣ
    /* m */ { 0x3161, 0x3161 }, // ㅡ
    /* n */ { 0x315C, 0x315C }, // ㅜ
    /* o */ { 0x3150, 0x3152 }, // ㅐ / ㅒ
    /* p */ { 0x3154, 0x3156 }, // ㅔ / ㅖ
    /* q */ { 0x3142, 0x3143 }, // ㅂ / ㅃ
    /* r */ { 0x3131, 0x3132 }, // ㄱ / ㄲ
    /* s */ { 0x3134, 0x3134 }, // ㄴ
    /* t */ { 0x3145, 0x3146 }, // ㅅ / ㅆ
    /* u */ { 0x3155, 0x3155 }, // ㅕ
    /* v */ { 0x314D, 0x314D }, // ㅍ
    /* w */ { 0x3148, 0x3149 }, // ㅈ / ㅉ
    /* x */ { 0x314C, 0x314C }, // ㅌ
    /* y */ { 0x315B, 0x315B }, // ㅛ
    /* z */ { 0x314B, 0x314B }, // ㅋ
  };
  if (qwerty_lower < 'a' || qwerty_lower > 'z') return 0;
  return map[qwerty_lower - 'a'][shift ? 1 : 0];
}
