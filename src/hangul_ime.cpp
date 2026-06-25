#include "hangul_ime.h"

// =====================================================================
// Hangul Jamo Tables
// =====================================================================
// Compat-block jamo (U+3131..U+3163)에서 internal index로 변환.
// 인덱스 의미:
//   cho  0..18 = ㄱㄲㄴㄷㄸㄹㅁㅂㅃㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎ
//   jung 0..20 = ㅏㅐㅑㅒㅓㅔㅕㅖㅗㅘㅙㅚㅛㅜㅝㅞㅟㅠㅡㅢㅣ
//   jong 0..27 = (없음) ㄱㄲㄳㄴㄵㄶㄷㄹㄺㄻㄼㄽㄾㄿㅀㅁㅂㅄㅅㅆㅇㅈㅊㅋㅌㅍㅎ

// 인덱스 길이: 0x3163 - 0x3131 + 1 = 51
static constexpr int JAMO_TBL_BASE = 0x3131;
static constexpr int JAMO_TBL_LEN  = 0x3163 - 0x3131 + 1;

// 두벌식 입력에서 나올 수 있는 모든 자모에 대해 cho/jung/jong 인덱스 정의.
// -1 = 해당 역할 불가, jong은 0 = 불가.
static const int8_t JAMO_TO_CHO[JAMO_TBL_LEN] = {
  /* 3131 ㄱ */  0,
  /* 3132 ㄲ */  1,
  /* 3133 ㄳ */ -1,
  /* 3134 ㄴ */  2,
  /* 3135 ㄵ */ -1,
  /* 3136 ㄶ */ -1,
  /* 3137 ㄷ */  3,
  /* 3138 ㄸ */  4,
  /* 3139 ㄹ */  5,
  /* 313A ㄺ */ -1,
  /* 313B ㄻ */ -1,
  /* 313C ㄼ */ -1,
  /* 313D ㄽ */ -1,
  /* 313E ㄾ */ -1,
  /* 313F ㄿ */ -1,
  /* 3140 ㅀ */ -1,
  /* 3141 ㅁ */  6,
  /* 3142 ㅂ */  7,
  /* 3143 ㅃ */  8,
  /* 3144 ㅄ */ -1,
  /* 3145 ㅅ */  9,
  /* 3146 ㅆ */ 10,
  /* 3147 ㅇ */ 11,
  /* 3148 ㅈ */ 12,
  /* 3149 ㅉ */ 13,
  /* 314A ㅊ */ 14,
  /* 314B ㅋ */ 15,
  /* 314C ㅌ */ 16,
  /* 314D ㅍ */ 17,
  /* 314E ㅎ */ 18,
  /* 314F ㅏ */ -1,
  /* 3150 ㅐ */ -1,
  /* 3151 ㅑ */ -1,
  /* 3152 ㅒ */ -1,
  /* 3153 ㅓ */ -1,
  /* 3154 ㅔ */ -1,
  /* 3155 ㅕ */ -1,
  /* 3156 ㅖ */ -1,
  /* 3157 ㅗ */ -1,
  /* 3158 ㅘ */ -1,
  /* 3159 ㅙ */ -1,
  /* 315A ㅚ */ -1,
  /* 315B ㅛ */ -1,
  /* 315C ㅜ */ -1,
  /* 315D ㅝ */ -1,
  /* 315E ㅞ */ -1,
  /* 315F ㅟ */ -1,
  /* 3160 ㅠ */ -1,
  /* 3161 ㅡ */ -1,
  /* 3162 ㅢ */ -1,
  /* 3163 ㅣ */ -1,
};

static const int8_t JAMO_TO_JUNG[JAMO_TBL_LEN] = {
  /* ㄱ..ㅎ */ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
  /* 314F ㅏ */  0,
  /* 3150 ㅐ */  1,
  /* 3151 ㅑ */  2,
  /* 3152 ㅒ */  3,
  /* 3153 ㅓ */  4,
  /* 3154 ㅔ */  5,
  /* 3155 ㅕ */  6,
  /* 3156 ㅖ */  7,
  /* 3157 ㅗ */  8,
  /* 3158 ㅘ */  9,
  /* 3159 ㅙ */ 10,
  /* 315A ㅚ */ 11,
  /* 315B ㅛ */ 12,
  /* 315C ㅜ */ 13,
  /* 315D ㅝ */ 14,
  /* 315E ㅞ */ 15,
  /* 315F ㅟ */ 16,
  /* 3160 ㅠ */ 17,
  /* 3161 ㅡ */ 18,
  /* 3162 ㅢ */ 19,
  /* 3163 ㅣ */ 20,
};

static const int8_t JAMO_TO_JONG[JAMO_TBL_LEN] = {
  /* 3131 ㄱ */  1,
  /* 3132 ㄲ */  2,
  /* 3133 ㄳ */  3,
  /* 3134 ㄴ */  4,
  /* 3135 ㄵ */  5,
  /* 3136 ㄶ */  6,
  /* 3137 ㄷ */  7,
  /* 3138 ㄸ */  0,    // ㄸ는 종성 불가
  /* 3139 ㄹ */  8,
  /* 313A ㄺ */  9,
  /* 313B ㄻ */ 10,
  /* 313C ㄼ */ 11,
  /* 313D ㄽ */ 12,
  /* 313E ㄾ */ 13,
  /* 313F ㄿ */ 14,
  /* 3140 ㅀ */ 15,
  /* 3141 ㅁ */ 16,
  /* 3142 ㅂ */ 17,
  /* 3143 ㅃ */  0,    // ㅃ는 종성 불가
  /* 3144 ㅄ */ 18,
  /* 3145 ㅅ */ 19,
  /* 3146 ㅆ */ 20,
  /* 3147 ㅇ */ 21,
  /* 3148 ㅈ */ 22,
  /* 3149 ㅉ */  0,    // ㅉ는 종성 불가
  /* 314A ㅊ */ 23,
  /* 314B ㅋ */ 24,
  /* 314C ㅌ */ 25,
  /* 314D ㅍ */ 26,
  /* 314E ㅎ */ 27,
  /* 모음들 */ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};

// 종성 분리 시 다음 음절의 초성 인덱스로 매핑. 0(없음)=초성 -1.
// 복합 종성은 별도 분해 후 두 번째 부분의 단일 종성으로 매핑한 결과를 사용.
// 즉 이 표는 "단일 종성" → "초성 인덱스".
static const int8_t SINGLE_JONG_TO_CHO[28] = {
  -1,  // 0  (none)
   0,  // 1  ㄱ → cho ㄱ(0)
   1,  // 2  ㄲ → cho ㄲ(1)
  -1,  // 3  ㄳ (compound; not single)
   2,  // 4  ㄴ → cho ㄴ(2)
  -1,  // 5  ㄵ (compound)
  -1,  // 6  ㄶ (compound)
   3,  // 7  ㄷ → cho ㄷ(3)
   5,  // 8  ㄹ → cho ㄹ(5)
  -1,  // 9  ㄺ
  -1,  //10  ㄻ
  -1,  //11  ㄼ
  -1,  //12  ㄽ
  -1,  //13  ㄾ
  -1,  //14  ㄿ
  -1,  //15  ㅀ
   6,  //16  ㅁ → cho ㅁ(6)
   7,  //17  ㅂ → cho ㅂ(7)
  -1,  //18  ㅄ
   9,  //19  ㅅ → cho ㅅ(9)
  10,  //20  ㅆ → cho ㅆ(10)
  11,  //21  ㅇ → cho ㅇ(11)
  12,  //22  ㅈ → cho ㅈ(12)
  14,  //23  ㅊ → cho ㅊ(14)
  15,  //24  ㅋ → cho ㅋ(15)
  16,  //25  ㅌ → cho ㅌ(16)
  17,  //26  ㅍ → cho ㅍ(17)
  18,  //27  ㅎ → cho ㅎ(18)
};

// 복합 종성: 두 단일 종성을 합칠 때. 0 = 합성 불가.
// 표는 (jong_left, jong_right) → compound_jong_index
static int8_t combine_jong(int8_t left, int8_t right) {
  // left는 이미 음절에 들어있는 단일 종성, right는 새로 들어온 종성
  if (left == 1 /*ㄱ*/ && right == 19 /*ㅅ*/) return 3;   // ㄳ
  if (left == 4 /*ㄴ*/ && right == 22 /*ㅈ*/) return 5;   // ㄵ
  if (left == 4 /*ㄴ*/ && right == 27 /*ㅎ*/) return 6;   // ㄶ
  if (left == 8 /*ㄹ*/ && right == 1  /*ㄱ*/) return 9;   // ㄺ
  if (left == 8 /*ㄹ*/ && right == 16 /*ㅁ*/) return 10;  // ㄻ
  if (left == 8 /*ㄹ*/ && right == 17 /*ㅂ*/) return 11;  // ㄼ
  if (left == 8 /*ㄹ*/ && right == 19 /*ㅅ*/) return 12;  // ㄽ
  if (left == 8 /*ㄹ*/ && right == 25 /*ㅌ*/) return 13;  // ㄾ
  if (left == 8 /*ㄹ*/ && right == 26 /*ㅍ*/) return 14;  // ㄿ
  if (left == 8 /*ㄹ*/ && right == 27 /*ㅎ*/) return 15;  // ㅀ
  if (left == 17 /*ㅂ*/ && right == 19 /*ㅅ*/) return 18; // ㅄ
  return 0;
}

// 복합 종성 분해: 분해 가능한 종성 → (앞쪽 단일 종성, 뒤쪽 단일 종성).
// 분해 불가면 left=0 반환.
struct DecompJong { int8_t left; int8_t right; };
static DecompJong decompose_jong(int8_t j) {
  switch (j) {
    case 3:  return {1, 19};   // ㄳ = ㄱ+ㅅ
    case 5:  return {4, 22};   // ㄵ = ㄴ+ㅈ
    case 6:  return {4, 27};   // ㄶ = ㄴ+ㅎ
    case 9:  return {8, 1};    // ㄺ = ㄹ+ㄱ
    case 10: return {8, 16};   // ㄻ = ㄹ+ㅁ
    case 11: return {8, 17};   // ㄼ = ㄹ+ㅂ
    case 12: return {8, 19};   // ㄽ = ㄹ+ㅅ
    case 13: return {8, 25};   // ㄾ = ㄹ+ㅌ
    case 14: return {8, 26};   // ㄿ = ㄹ+ㅍ
    case 15: return {8, 27};   // ㅀ = ㄹ+ㅎ
    case 18: return {17, 19};  // ㅄ = ㅂ+ㅅ
    default: return {0, 0};
  }
}

// 복합 중성: 이미 set된 jung에 새 vowel jung을 합칠 수 있나?
// jung_left, jung_right → compound jung index. -1 = 합성 불가.
static int8_t combine_jung(int8_t left, int8_t right) {
  if (left == 8  /*ㅗ*/ && right == 0  /*ㅏ*/) return 9;   // ㅘ
  if (left == 8  /*ㅗ*/ && right == 1  /*ㅐ*/) return 10;  // ㅙ
  if (left == 8  /*ㅗ*/ && right == 20 /*ㅣ*/) return 11;  // ㅚ
  if (left == 13 /*ㅜ*/ && right == 4  /*ㅓ*/) return 14;  // ㅝ
  if (left == 13 /*ㅜ*/ && right == 5  /*ㅔ*/) return 15;  // ㅞ
  if (left == 13 /*ㅜ*/ && right == 20 /*ㅣ*/) return 16;  // ㅟ
  if (left == 18 /*ㅡ*/ && right == 20 /*ㅣ*/) return 19;  // ㅢ
  return -1;
}

// 복합 중성 분해: 분해 가능한 jung → 앞쪽 단일 jung. -1 = 분해 불가.
static int8_t decompose_jung(int8_t j) {
  switch (j) {
    case 9:  return 8;   // ㅘ → ㅗ
    case 10: return 8;   // ㅙ → ㅗ
    case 11: return 8;   // ㅚ → ㅗ
    case 14: return 13;  // ㅝ → ㅜ
    case 15: return 13;  // ㅞ → ㅜ
    case 16: return 13;  // ㅟ → ㅜ
    case 19: return 18;  // ㅢ → ㅡ
    default: return -1;
  }
}

// 단독 cho를 표시할 때 쓰는 compat block 자모 (cho index → compat codepoint)
static const uint16_t CHO_TO_COMPAT[19] = {
  0x3131, 0x3132, 0x3134, 0x3137, 0x3138, 0x3139, 0x3141, 0x3142, 0x3143,
  0x3145, 0x3146, 0x3147, 0x3148, 0x3149, 0x314A, 0x314B, 0x314C, 0x314D, 0x314E
};
// 단독 jung 표시용 (jung index → compat codepoint)
static const uint16_t JUNG_TO_COMPAT[21] = {
  0x314F, 0x3150, 0x3151, 0x3152, 0x3153, 0x3154, 0x3155, 0x3156,
  0x3157, 0x3158, 0x3159, 0x315A, 0x315B, 0x315C, 0x315D, 0x315E,
  0x315F, 0x3160, 0x3161, 0x3162, 0x3163
};

// =====================================================================
// HangulIME implementation
// =====================================================================

void HangulIME::reset() {
  cho_ = -1;
  jung_ = -1;
  jong_ = 0;
}

String HangulIME::encode_utf8(uint32_t cp) {
  String s;
  if (cp < 0x80) {
    s += (char)cp;
  } else if (cp < 0x800) {
    s += (char)(0xC0 | (cp >> 6));
    s += (char)(0x80 | (cp & 0x3F));
  } else if (cp < 0x10000) {
    s += (char)(0xE0 | (cp >> 12));
    s += (char)(0x80 | ((cp >> 6) & 0x3F));
    s += (char)(0x80 | (cp & 0x3F));
  } else {
    s += (char)(0xF0 | (cp >> 18));
    s += (char)(0x80 | ((cp >> 12) & 0x3F));
    s += (char)(0x80 | ((cp >> 6) & 0x3F));
    s += (char)(0x80 | (cp & 0x3F));
  }
  return s;
}

// cho + jung [+ jong] → UTF-8. emit/commit 경로에서 호출되므로 정직하게:
// 자음만이면 compat 자음, 모음만이면 compat 모음 그대로 반환.
String HangulIME::compose_syllable(int8_t cho, int8_t jung, int8_t jong) {
  if (cho >= 0 && jung >= 0) {
    uint32_t cp = 0xAC00 + (cho * 21 + jung) * 28 + jong;
    return encode_utf8(cp);
  }
  if (cho >= 0) return encode_utf8(CHO_TO_COMPAT[cho]);
  if (jung >= 0) return encode_utf8(JUNG_TO_COMPAT[jung]);
  return String();
}

String HangulIME::preview() const {
  // 자음/모음 단독은 compat block 자모 (U+3131..U+3163)를 그대로 반환.
  // 디스플레이가 자체 자모 비트맵 폰트로 그림 (jamo_font.h).
  return compose_syllable(cho_, jung_, jong_);
}

String HangulIME::commit_all() {
  String out = preview();
  reset();
  return out;
}

String HangulIME::input_jamo(uint16_t jamo_cp) {
  if (jamo_cp < JAMO_TBL_BASE || jamo_cp >= JAMO_TBL_BASE + JAMO_TBL_LEN) {
    return String();
  }
  int idx = jamo_cp - JAMO_TBL_BASE;
  int8_t cho_i  = JAMO_TO_CHO[idx];
  int8_t jung_i = JAMO_TO_JUNG[idx];
  int8_t jong_i = JAMO_TO_JONG[idx];

  bool is_vowel = (jung_i >= 0);
  String emit;

  if (is_vowel) {
    // 자음 없이 모음 → 단독 모음을 emit하고 상태 유지 안 함
    if (cho_ < 0) {
      emit = compose_syllable(-1, jung_i, 0);
      reset();
      return emit;
    }
    // cho만 있고 jung 없음 → jung 설정
    if (jung_ < 0) {
      jung_ = jung_i;
      return String();
    }
    // 종성 없음: 복합 모음 시도
    if (jong_ == 0) {
      int8_t combo = combine_jung(jung_, jung_i);
      if (combo >= 0) {
        jung_ = combo;
        return String();
      }
      // 합성 불가 → 현재 음절 확정, 모음만 단독으로 emit
      emit = compose_syllable(cho_, jung_, 0);
      emit += compose_syllable(-1, jung_i, 0);
      reset();
      return emit;
    }
    // 종성 있음 → split: 종성을 다음 음절의 초성으로 이동
    DecompJong d = decompose_jong(jong_);
    if (d.left != 0) {
      // 복합 종성: 첫 부분은 그대로 종성으로 남기고, 뒷부분이 다음 cho
      emit = compose_syllable(cho_, jung_, d.left);
      int8_t new_cho = SINGLE_JONG_TO_CHO[d.right];
      cho_ = (new_cho >= 0) ? new_cho : -1;
      jung_ = jung_i;
      jong_ = 0;
      return emit;
    }
    // 단일 종성: 종성을 통째로 떼서 다음 cho로
    int8_t new_cho = SINGLE_JONG_TO_CHO[jong_];
    emit = compose_syllable(cho_, jung_, 0);
    cho_ = (new_cho >= 0) ? new_cho : -1;
    jung_ = jung_i;
    jong_ = 0;
    return emit;
  }

  // 자음 입력
  if (cho_ < 0) {
    // 새 음절 시작
    if (cho_i >= 0) {
      cho_ = cho_i;
    } else {
      // cho 불가능한 자음 (복합 자모 등) - 단독 emit
      emit = encode_utf8(jamo_cp);
    }
    return emit;
  }
  if (jung_ < 0) {
    // cho만 있고 다음도 자음 → cho 단독 확정, 새 음절 시작
    emit = compose_syllable(cho_, -1, 0);
    cho_ = cho_i;
    jung_ = -1;
    jong_ = 0;
    if (cho_ < 0) {                 // cho 불가능 자음이면 단독 emit
      emit += encode_utf8(jamo_cp);
    }
    return emit;
  }
  // cho + jung 설정됨
  if (jong_ == 0) {
    if (jong_i > 0) {
      jong_ = jong_i;
      return String();
    }
    // 종성 불가능 자음 (ㄸㅃㅉ) → 음절 확정 후 새 cho로 시작
    emit = compose_syllable(cho_, jung_, 0);
    cho_ = cho_i;
    jung_ = -1;
    jong_ = 0;
    if (cho_ < 0) emit += encode_utf8(jamo_cp);
    return emit;
  }
  // 종성 이미 있음: 복합 종성 시도
  if (jong_i > 0) {
    int8_t combo = combine_jong(jong_, jong_i);
    if (combo > 0) {
      jong_ = combo;
      return String();
    }
  }
  // 종성 합성 불가 → 음절 확정 후 새 음절 시작
  emit = compose_syllable(cho_, jung_, jong_);
  cho_ = cho_i;
  jung_ = -1;
  jong_ = 0;
  if (cho_ < 0) emit += encode_utf8(jamo_cp);
  return emit;
}

HangulIME::BackspaceResult HangulIME::backspace() {
  if (jong_ > 0) {
    int8_t simple = decompose_jong(jong_).left;
    jong_ = (simple != 0) ? simple : 0;
    return {true, false};
  }
  if (jung_ >= 0) {
    int8_t simple = decompose_jung(jung_);
    jung_ = (simple >= 0) ? simple : -1;
    return {true, false};
  }
  if (cho_ >= 0) {
    cho_ = -1;
    return {true, false};
  }
  return {false, true};
}
