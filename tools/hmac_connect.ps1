#Requires -Version 5.1
<#
.SYNOPSIS
    Connect to the MeshCom HMAC console (port 2323).
.EXAMPLE
    .\hmac_connect.ps1 192.168.1.100
    .\hmac_connect.ps1 192.168.1.100 MeinPasswort
    .\hmac_connect.ps1 192.168.1.100 MeinPasswort 2323
#>
param(
    [Parameter(Mandatory)][string]$Host,
    [string]$Password = "",
    [int]$Port = 2323
)

$tcp = [Net.Sockets.TcpClient]::new($Host, $Port)
$stream = $tcp.GetStream()
$rd = [IO.StreamReader]::new($stream)
$wr = [IO.StreamWriter]::new($stream)
$wr.AutoFlush = $true

if ($Password -ne "") {
    # Read "NONCE: <32 hex chars>"
    $nonceHex = ($rd.ReadLine() -split " ")[1].Trim()

    # Hex string -> byte[] (PS 5.1 compatible — no FromHexString)
    $nonce = [byte[]]@(0..($nonceHex.Length / 2 - 1) | ForEach-Object {
        [Convert]::ToByte($nonceHex.Substring($_ * 2, 2), 16)
    })

    # HMAC-SHA256(key=password, data=nonce)
    $hmac = [Security.Cryptography.HMACSHA256]::new(
        [Text.Encoding]::UTF8.GetBytes($Password))
    $hash = $hmac.ComputeHash($nonce)

    # byte[] -> lowercase hex (PS 5.1 compatible — no ToHexString)
    $response = [BitConverter]::ToString($hash).Replace('-', '').ToLower()

    $wr.WriteLine($response)

    $result = $rd.ReadLine().Trim()
    if ($result -ne "OK") {
        Write-Error "Authentication failed: $result"
        $tcp.Close(); exit 1
    }
    Write-Host "Authentication OK." -ForegroundColor Green
}

# Read banner
$stream.ReadTimeout = 800
try {
    while ($true) {
        $line = $rd.ReadLine()
        if ($null -eq $line) { break }
        Write-Host $line
    }
} catch { }
$stream.ReadTimeout = [Threading.Timeout]::Infinite

Write-Host "--- Connected (Ctrl+C to quit) ---" -ForegroundColor Cyan

# Receive loop (background job)
$job = [PowerShell]::Create()
$null = $job.AddScript({
    param($rd)
    try {
        while ($true) {
            $line = $rd.ReadLine()
            if ($null -eq $line) { break }
            Write-Host $line
        }
    } catch { }
}).AddArgument($rd)
$handle = $job.BeginInvoke()

# Send loop (foreground)
try {
    while ($true) {
        $line = Read-Host
        $wr.WriteLine($line)
    }
} catch [Management.Automation.PipelineStoppedException] {
    # Ctrl+C
} finally {
    $job.Stop()
    $tcp.Close()
}
