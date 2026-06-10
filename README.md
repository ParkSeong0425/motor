# ⚙️ Motor Control System (STM32 & FreeRTOS)

본 프로젝트는 **STM32 MCU**와 **FreeRTOS(실시간 운영체제)**를 기반으로 구축되었으며, 고성능 서보 모터(**AIMotor**)를 **TCP/IP 네트워크 및 RS485(Modbus RTU) 직렬 통신**을 통해 원격 및 직렬로 제어하는 통합 임베디드 제어 시스템입니다.

별도의 프로그램 설치 없이, **PC의 PowerShell 환경**을 통한 원격 제어 및 보드 내장 **RS485 CLI(Command Line Interface) 터미널** 환경을 통해 하드웨어를 정밀하게 제어하고 모니터링할 수 있도록 구성했습니다.

---

## 🚀 주요 기능 (Key Features)

* **FreeRTOS 멀티태스킹 아키텍처**: 내부 Task 간 병렬 처리를 구현하여 모터 제어 및 통신 데이터 실시간 동기화
  * `MotorTask`: 가·감속 모션 프로파일 연산 및 Queue(큐) 기반 실시간 명령 처리
  * `CliTask` / `TcpTask`: 사용자 직렬 터미널 및 원격 TCP 소켓 스트림 분석 후 명령 하달
* **RS485 Modbus RTU 프로토콜 제어**: AIMotor 전용 레지스터 맵(위치, 속도, 구동 가속도, 제어 모드 등)을 활용한 고정밀 서보 제어 구현
* **실시간 속도 및 방향 제어**: 위치(mm) 기반 이동 제어, RPM 설정, 정회전/역회전 제어 기능
* **강력한 CLI 터미널 환경**: `huart6`를 활용한 직관적인 명령어 인터페이스 환경 제공으로 편리한 현장 디버깅 지원

---

## 🛠️ 통신 사양

| 항목 | TCP 설정 | RS485 설정 |
| :--- | :--- | :--- |
| **연결 방식** | TCP/IP Server (보드) / Client (PC) | Half-Duplex Serial 통신 채널 (`huart5`) |
| **물리 계층 Pin** | RJ-45 Ethernet 포트 | PD13 (Direction Pin), PD5(TX), PD6(RX) |
| **통신 속도** | 10/100 Mbps (Auto-Negotiation) | **115,200 bps** (Baudrate) |
| **프로토콜** | Modbus TCP (또는 Custom TCP String) | **Modbus RTU (CRC-16 검증 포함)** |

---

## 🏗️ 소스 코드 구조

├── Src/
│   ├── freertos.c      # Multi-Tasking (MotorTask, CliTask, TcpTask) 및 Message Queue 구조 설계
│   ├── motor.c         # AIMotor 구동 프로파일 제어 및 Modbus 레지스터 송수신 추상화
│   ├── rs485.c         # Half-Duplex 방향 제어 및 CRC-16 계산 기법 기반 저수준 드라이버
│   └── cli.c           # 현장 테스트 및 디버깅용 직렬 통신 Command 라인 인터페이스 엔진


---

## 💻 사용 방법 (Quick Start)

### 1. 하드웨어 배선 및 전원 연결
* 모터와 제어 PC(또는 MCU)의 RS485/TCP 배선을 연결합니다.
* 보드 내부 `PD13` 핀이 하프 듀플렉스(Half-Duplex) 송수신 방향을 자동으로 제어합니다.

### 2. CLI 직렬 터미널 제어 (현장 제어)
보드의 `UART6` 포트에 시리얼 터미널을 연결(115200bps)한 뒤, 아래 명령어를 통해 모터를 제어할 수 있습니다.
* **모터 구동**: `motor move <target_mm> <speed>`
* **모터 정지**: `motor stop`
* **비상 정지 / 서보 전원 해제**: `motor estop` / `motor release`

### 3. PowerShell 원격 제어 (네트워크 제어)
소스 코드에서 목적지 IP 주소와 통신 포트(Port)를 설정한 후, PC의 PowerShell 환경에서 아래 명령어를 실행하여 바탕화면의 `stm_tcp.ps1` 스크립트를 즉시 구동합니다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -Command "$p=[Environment]::GetFolderPath('De
