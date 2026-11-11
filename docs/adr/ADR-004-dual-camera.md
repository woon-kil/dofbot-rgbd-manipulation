# ADR-004: Gemini 335L + DaBai 듀얼 카메라

상태: ACCEPTED_FOR_DESIGN / NOT_HARDWARE_VALIDATED.

결정: Gemini 335L을 **필수 전역 eye-to-hand 카메라**, DaBai를 **필수 근접 eye-in-hand 카메라**로 사용. 기본은 순차 관측, 실시간 3D fusion은 도입하지 않는다.

근거: 고정 카메라는 전체 장면 및 결과 검증, 말단 카메라는 근접 재관측을 담당한다. Gemini는 사용자가 이미 보유하고 고정 위치 이동 불가. 별도 base 기준 보정이 필요하며 보이지 않는 작업영역을 임의 추정하지 않는다.

대안: DaBai 단독은 별도 fallback 실험만 허용하고 듀얼 PASS 불가. Gemini 단독은 근접 확인 부족. 동시 융합은 동기화/간섭/오차 예산 확보 후 검토.

Tradeoff: 보정 2개·시각/가림 관리 비용 증가. 두 센서 결과 불일치 시 자동 명령 거부.
