param(
    [string]$HostName = "192.168.0.104",
    [string]$User = "obr",
    [string]$RemoteDir = "/home/obr/OBR2026K",
    [string]$Target = "robot_test",
    [string]$KeyPath = "$env:USERPROFILE\.ssh\obr_raspberry",
    [switch]$Run,
    [switch]$NoRun
)

$ErrorActionPreference = "Stop"

function Invoke-Checked {
    param(
        [string]$Command,
        [string[]]$Arguments
    )

    & $Command @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed: $Command $($Arguments -join ' ')"
    }
}

$workspace = Split-Path -Parent $PSScriptRoot
$remote = "$User@$HostName"
$remoteBuild = "$RemoteDir/build"
$sshArgs = @()
$scpArgs = @()

if (Test-Path $KeyPath) {
    $sshArgs += @("-i", $KeyPath)
    $scpArgs += @("-i", $KeyPath)
}

Write-Host "Deploying to ${remote}:$RemoteDir"

Invoke-Checked ssh @($sshArgs + @($remote, "mkdir -p '$RemoteDir' '$remoteBuild'"))
Invoke-Checked scp @(
    $scpArgs +
    "$workspace/CMakeLists.txt",
    "${remote}:$RemoteDir/CMakeLists.txt"
)
Invoke-Checked scp @(
    $scpArgs +
    "-r",
    "$workspace/src",
    "${remote}:$RemoteDir/"
)
Invoke-Checked ssh @(
    $sshArgs +
    $remote,
    "cd '$RemoteDir' && cmake -S . -B build && cmake --build build"
)

Write-Host "Deploy complete: ${remote}:$remoteBuild/$Target"

if (-not $NoRun) {
    Invoke-Checked ssh @($sshArgs + @($remote, "cd '$RemoteDir' && ./build/$Target"))
} else {
    Write-Host "Robot was deployed but is not running."
    Write-Host "Dashboard URL after starting: http://${HostName}:8080"
}
