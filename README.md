# X Motor Exam

STM32F446 기반 AIMotor 제어 테스트 프로젝트입니다.

## 기능

* AIMotor RS485 제어
* 원점 ↔ 저장 위치 반복 이동
* DI3 Emergency Stop 제어
* FRAM 위치 저장
* MAC EEPROM 읽기
* W6100 TCP 서버 테스트

## Network

```text
IP   : 172.20.0.192
MASK : 255.255.0.0
PORT : 5000
MAC  : EEPROM
```

## CLI

```text
set
clear
save out 100
run 1 10
stop
estop
release
```

## TCP

현재 TCP는 요청-응답 방식입니다.

```text
01Q       상태 확인
01STOP    정지
01S       긴급정지
01D       해제
```

## Status

완료:

```text
MAC EEPROM 읽기
FRAM 확인
ping 성공
TCP 01Q 응답 확인
CLI 반복 동작 구현
DI3 긴급정지 적용
```

추가 예정:

```text
TCP 자동 로그 출력
DI2 원점 센서 최종 검증
속도/가감속 튜닝
```
