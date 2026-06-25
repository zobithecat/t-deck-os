#pragma once
#include <Arduino.h>
#include <stdint.h>

// 두벌식 한글 조합기. 입력은 compat-block 자모(U+3131..U+3163) 코드포인트.
//
// 상태: 현재 조합 중인 음절의 (초성, 중성, 종성) 인덱스.
//   cho:  -1 또는 0..18
//   jung: -1 또는 0..20
//   jong:  0(없음) 또는 1..27
//
// input_jamo()는 새 자모를 받아 commit해야 할 텍스트(없거나 한 음절 또는 두 음절)를
// UTF-8 String으로 반환한다. 호출자는 이 반환값을 메인 버퍼에 append하고, 미리보기는
// preview()로 얻어서 끝에 붙여 렌더링하면 된다.
class HangulIME {
public:
  HangulIME() { reset(); }

  // 새 compat-block 자모 코드포인트를 입력. 반환: commit할 UTF-8 텍스트.
  String input_jamo(uint16_t jamo_cp);

  // 미리보기: 현재 조합 중인 음절을 UTF-8로 반환. 없으면 빈 문자열.
  String preview() const;

  // 백스페이스: 조합 분해 또는 메인 버퍼 마지막 글자 삭제 신호.
  struct BackspaceResult {
    bool consumed;              // IME가 한 단계 분해를 처리함
    bool remove_buffer_char;    // 호출자가 메인 버퍼의 마지막 UTF-8 글자를 지워야 함
  };
  BackspaceResult backspace();

  // 현재 조합을 강제 확정 (스페이스/엔터/모드전환 등에서 호출). 반환: commit할 UTF-8.
  String commit_all();

  // 빈 상태인가?
  bool is_composing() const { return cho_ >= 0 || jung_ >= 0; }

  void reset();

private:
  int8_t cho_;   // -1..18
  int8_t jung_;  // -1..20
  int8_t jong_;  // 0..27

  static String encode_utf8(uint32_t cp);
  static String compose_syllable(int8_t cho, int8_t jung, int8_t jong);
};
