param(
    [string]$HostName = "raspberrypi.local",
    [string]$User = "obr",
    [string]$RemoteDir = "/home/obr/OBR2026K",
    [string]$Target = "robot_test",
    [string]$KeyPath = "$env:USERPROFILE\.ssh\obr_raspberry",
    [string]$ServiceName = "obr-robot",
    [switch]$Service,
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
Invoke-Checked scp @(
    $scpArgs +
    "-r",
    "$workspace/include",
    "${remote}:$RemoteDir/"
)
Invoke-Checked ssh @(
    $sshArgs +
    $remote,
    "cd '$RemoteDir' && cmake -S . -B build && cmake --build build"
)

Write-Host "Deploy complete: ${remote}:$remoteBuild/$Target"

if ($NoRun) {
    Write-Host "Robot was deployed but is not running."
    Write-Host "Dashboard URL after starting: http://${HostName}:8080"
} elseif ($Service) {
    Invoke-Checked ssh @($sshArgs + @($remote, "sudo systemctl restart $ServiceName.service && sudo systemctl status $ServiceName.service --no-pager"))
    Write-Host "Service restarted. Dashboard URL: http://${HostName}:8080"
} elseif ($Run) {
    Invoke-Checked ssh @($sshArgs + @($remote, "cd '$RemoteDir' && ./build/$Target"))
} else {
    $startCommand = "if systemctl cat $ServiceName.service >/dev/null 2>&1; then sudo systemctl restart $ServiceName.service && sudo systemctl status $ServiceName.service --no-pager; else cd '$RemoteDir' && ./build/$Target; fi"
    Invoke-Checked ssh @($sshArgs + @($remote, $startCommand))
    Write-Host "Dashboard URL: http://${HostName}:8080"
}
