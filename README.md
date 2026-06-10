# stm_tcp.ps1
# 바탕화면에서 실행되어 STM32 보드로 TCP 제어 명령을 전송하는 스크립트

Clear-Host
Write-Host "=============================================" -ForegroundColor Cyan
Write-Host "   STM32 TCP Motor Control Remote Terminal    " -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan

# 1. 통신 환경 설정 (보드 IP 및 포트 설정)
$TargetIP = "192.168.1.100"   # STM32 보드의 IP 주소에 맞게 변경하세요
$TargetPort = 5000            # STM32 TCP Server 포트 번호

Write-Host "[INFO] 연결 대상 보드: $TargetIP : $TargetPort" -ForegroundColor Yellow

# 2. 보드와 TCP 소켓 연결 시도
try {
    $socket = New-Object System.Net.Sockets.TcpClient
    $ConnectResult = $socket.BeginConnect($TargetIP, $TargetPort, $null, $null)
    $Wait = $ConnectResult.AsyncWaitHandle.WaitOne(2000, $true) # 2초 타임아웃
    
    if (-not $Wait) {
        throw "연결 시간 초과 (Timeout)"
    }
    $socket.EndConnect($ConnectResult)
    $stream = $socket.GetStream()
    Write-Host "[SUCCESS] STM32 모터 제어 보드에 성공적으로 연결되었습니다." -ForegroundColor Green
}
catch {
    Write-Host "[ERROR] 보드 연결 실패: $_" -ForegroundColor Red
    Write-Host "[HINT] 배선 및 STM32 네트워크 설정을 확인하세요." -ForegroundColor Yellow
    Exit
}

# 3. 원격 모터 제어 루프
$running = $true
while ($running) {
    Write-Host "`n---------------------------------------------"
    Write-Host " 1. 모터 구동 (MOVE)"
    Write-Host " 2. 모터 정지 (STOP)"
    Write-Host " 3. 비상 정지 (ESTOP)"
    Write-Host " 4. 서보 해제 (RELEASE)"
    Write-Host " 5. 종료 (EXIT)"
    Write-Host "---------------------------------------------"
    $choice = Read-Host "원하는 제어 명령 번호를 입력하세요"

    switch ($choice) {
        "1" {
            $mm = Read-Host "이동 거리 입력 (mm)"
            $speed = Read-Host "속도 비율 입력 (1~100 %)"
            
            # 텍스트 형식 패킷 포맷 정의: "MOVE,<mm>,<speed>\n"
            $cmdText = "MOVE,$mm,$speed`n"
        }
        "2" {
            $cmdText = "STOP`n"
        }
        "3" {
            $cmdText = "ESTOP`n"
        }
        "4" {
            $cmdText = "RELEASE`n"
        }
        "5" {
            $cmdText = "EXIT`n"
            $running = $false
            continue
        }
        default {
            Write-Host "[WARN] 잘못된 입력입니다. 다시 선택하세요." -ForegroundColor Yellow
            continue
        }
    }

    # TCP 데이터 송신
    try {
        $sendData = [System.Text.Encoding]::ASCII.GetBytes($cmdText)
        $stream.Write($sendData, 0, $sendData.Length)
        Write-Host "[SEND] 전송된 명령: $($cmdText.TrimEnd())" -ForegroundColor Magenta
        
        # 보드로부터 응답 메시지 수신 (1024 바이트 버퍼)
        $buffer = New-Object Byte[] 1024
        $bytesRead = $stream.Read($buffer, 0, $buffer.Length)
        if ($bytesRead -gt 0) {
            $response = [System.Text.Encoding]::ASCII.GetString($buffer, 0, $bytesRead)
            Write-Host "[RECEIVE] 보드 상태 응답: $($response.TrimEnd())" -ForegroundColor Green
        }
    }
    catch {
        Write-Host "[ERROR] 데이터 송수신 중 오류 발생: $_" -ForegroundColor Red
        $running = $false
    }
}

# 4. 소켓 리소스 해제
$stream.Close()
$socket.Close()
Write-Host "[INFO] 원격 제어 터미널 세션을 종료합니다." -ForegroundColor Cyan
