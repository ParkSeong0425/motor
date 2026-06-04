$ip = "172.20.0.192"
$port = 5000

$client = New-Object System.Net.Sockets.TcpClient
$client.Connect($ip, $port)
$stream = $client.GetStream()

Write-Host "Connected to STM TCP server ${ip}:${port}"
Write-Host ""
Write-Host "Commands:"
Write-Host "  01_C"
Write-Host "    Read status"
Write-Host ""
Write-Host "  01H"
Write-Host "    Enable servo and save current sensor position as software home"
Write-Host ""
Write-Host "  01M_position_speed_startDelayMs_returnWaitMs"
Write-Host "    Move to logical absolute lift position"
Write-Host "    Example: 01M_5000_20_1000_500"
Write-Host ""
Write-Host "  01a_AIM_mode_string_rack_sep_position_speed"
Write-Host "    AIMotor command-compatible move. Rack/sep/string/mode are ignored for one motor."
Write-Host "    Example: 01a_AIM_ABS_LIFT_0_0_5000_20"
Write-Host ""
Write-Host "  01_S"
Write-Host "    Stop motion. Servo remains enabled."
Write-Host ""
Write-Host "  01D"
Write-Host "    Release pause/e-stop: DI3 OFF, DI2 alarm reset, servo enable ON"
Write-Host ""
Write-Host "  exit"
Write-Host ""

function Send-Cmd($text) {
    $cmd = [System.Text.Encoding]::ASCII.GetBytes($text + "`r`n")
    $stream.Write($cmd, 0, $cmd.Length)

    $buffer = New-Object byte[] 2048
    $count = $stream.Read($buffer, 0, $buffer.Length)

    [System.Text.Encoding]::ASCII.GetString($buffer, 0, $count)
}

try {
    while ($true) {
        $line = Read-Host "STM"

        if ($line -eq "exit") {
            break
        }

        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }

        try {
            $res = Send-Cmd $line
            Write-Host $res
        }
        catch {
            Write-Host "TCP error:"
            Write-Host $_
            break
        }
    }
}
finally {
    if ($stream) { $stream.Close() }
    if ($client) { $client.Close() }
    Write-Host "Disconnected"
}