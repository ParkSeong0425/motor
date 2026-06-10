# ⚙️ High-Performance Motor Control System (STM32 & FreeRTOS)

본 프로젝트는 **STM32 MCU**와 **FreeRTOS(실시간 운영체제)**를 기반으로 구축되었으며, 차세대 산업용 고성능 서보 모터(**AIMotor**)를 **TCP/IP 네트워크 및 RS485(Modbus RTU) 직렬 통신**을 통해 원격 및 직렬로 제어하는 통합 임베디드 제어 시스템입니다.

별도의 복잡한 제어 프로그램 설치 없이, **PC의 PowerShell 환경** 또는 독립적인 **RS485 CLI(Command Line Interface) 터미널** 환경을 통해 시스템 가동 상태를 모니터링하고 하드웨어를 정밀하게 제어할 수 있도록 설계되었습니다.

---

## 🚀 핵심 기능 (Key Features)

* **FreeRTOS 멀티태스킹 아키텍처**: 내부 Task 간 병렬 처리를 구현하여 모터 제어 주기 및 통신 데이터 정밀도 최적화
  * `MotorTask`: 가·감속 모션 프로파일 연산 및 큐(Queue) 기반 실시간 동기식 명령 처리
  * `CliTask` / `TcpTask`: 사용자 터미널 입력 및 원격 TCP 소켓 스트림 파싱 및 상태 피드백
* **RS485 Modbus RTU 프로토콜 하드웨어 제어**: AIMotor 전용 레지스터 맵(위치, 속도, 구동 가속도, 제어 모드 등)을 통한 32비트 고정밀 서보 제어 구현
* **강력한 CLI 터미널 환경**: `huart6`를 활용한 전용 명령어 인터페이스 환경 제공으로 직관적인 현장 디버깅 지원
* **원격 TCP/IP 통신 인터페이스**: 네트워크망(Ethernet/Wi-Fi)을 통해 원격지에서도 지연 없이 구동 제어 및 모터 RPM/위치 트래킹 가능

---

## 🛠️ 하드웨어 및 시스템 통신 사양

| 항목 | TCP/IP 원격 제어 단 | RS485 (Modbus RTU) 모터 구동 단 |
| :--- | :--- | :--- |
| **연결 방식** | TCP/IP Server (보드) & Client (PC) | Half-Duplex RS485 직렬 포트 (`huart5`) |
| **물리 계층 Pin** | RJ-45 Ethernet 포트 | PD13 (DIR Pin), PD5(TX), PD6(RX) |
| **통신 속도** | 10/100 Mbps (Auto-Negotiation) | **115,200 bps** (Baudrate) |
| **프로토콜** | Custom TCP String Command | **Modbus RTU (With CRC16 검증)** |
| **주요 제어 대상** | PowerShell / 원격 소프트웨어 인터페이스 | 고정밀 산업용 서보 디바이스 (**AIMotor**) |

---

## 🏗️ 시스템 아키텍처 및 소
