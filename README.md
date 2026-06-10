# ⚙️ Motor Control 
TCP 통신과 RS485(Modbus) 통신을 이용하여 모터를 제어하는 프로젝트입니다. 별도의 프로그램 설치 없이 하드웨어를 원격 및 직렬로 제어할 수 있도록 구성했습니다.


## 🚀 주요 기능
* **TCP/IP 통신 제어**: 네트워크(Ethernet/Wi-Fi)를 통해 원격에서 모터 구동 및 상태 모니터링
* **RS485 직렬 통신**: Modbus RTU 프로토콜을 활용하여 다수의 모터 디바이스와 안정적인 데이터 송수신
* **실시간 속도 및 방향 제어**: RPM 설정, 정회전/역회전 제어 기능

## 🛠️ 통신 사양 (예시)
| 항목 | TCP 설정 | RS485 설정 |
| :--- | :--- | :--- |
| **연결 방식** | TCP/IP Client / Server | Serial (통신 포트) |
| **통신 속도** |             | 115200 bps (Baudrate) |
| **프로토콜** | Modbus TCP  | Modbus RTU |

## 💻 사용 방법
1. 모터와 제어 PC(또는 MCU)의 RS485/TCP 배선을 연결합니다.
2. 소스 코드에서 목적지 IP 주소와 통신 포트(Port/ComPort)를 설정합니다.
3. 제어 명령 코드를 실행하여 모터를 구동합니다.

## powershell 코드 

powershell -NoProfile -ExecutionPolicy Bypass -Command "$p=[Environment]::GetFolderPath('Desktop'); & (Join-Path $p 'stm_tcp.ps1')"
