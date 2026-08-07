param(
    [string]$HostName = "192.168.0.110",
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
$remoteStagingBuild = "$RemoteDir/.build-staging"
$remoteCameraPattern = "$RemoteDir/scripts/[c]amera_line_frame.py"
$remoteRunScript = "$RemoteDir/scripts/run_robot.sh"
$sshArgs = @()
$scpArgs = @()

if (Test-Path $KeyPath) {
    $sshArgs += @("-i", $KeyPath)
    $scpArgs += @("-i", $KeyPath)
} else {
    Write-Host "SSH key not found at $KeyPath. SSH may ask for the Raspberry password."
}

Write-Host "Deploying to ${remote}:$RemoteDir"

Invoke-Checked ssh @($sshArgs + @($remote, "mkdir -p '$RemoteDir' '$remoteBuild'"))

# Para o serviço antes de trocar código ou binário. Se o build falhar, o robô
# permanece parado e o executável válido anterior não é substituído.
Invoke-Checked ssh @($sshArgs + @($remote, "sudo systemctl stop '$ServiceName.service' >/dev/null 2>&1 || true"))
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
Invoke-Checked scp @(
    $scpArgs +
    "-r",
    "$workspace/scripts",
    "${remote}:$RemoteDir/"
)
$atomicBuildCommand = "cd '$RemoteDir' && cmake -S . -B '$remoteStagingBuild' && cmake --build '$remoteStagingBuild' --target '$Target' && test -s '$remoteStagingBuild/$Target' && install -m 755 '$remoteStagingBuild/$Target' '$remoteBuild/$Target.new' && mv -f '$remoteBuild/$Target.new' '$remoteBuild/$Target' && test -s '$remoteBuild/$Target'"
Invoke-Checked ssh @($sshArgs + @($remote, $atomicBuildCommand))

Write-Host "Deploy complete: ${remote}:$remoteBuild/$Target"

$stopOldCameraCommand = "pkill -f '$remoteCameraPattern' >/dev/null 2>&1 || true"
$prepareScriptsCommand = "cd '$RemoteDir' && find scripts -type f \( -name '*.sh' -o -name '*.service' \) -exec sed -i 's/\r$//' {} + && chmod +x '$remoteRunScript'"
$installServiceCommand = "$prepareScriptsCommand && sudo cp '$RemoteDir/scripts/$ServiceName.service' '/etc/systemd/system/$ServiceName.service' && sudo systemctl daemon-reload && sudo systemctl enable $ServiceName.service >/dev/null 2>&1"
$restartServiceCommand = "$installServiceCommand && sudo systemctl restart $ServiceName.service"
$statusServiceCommand = "sudo systemctl is-active --quiet $ServiceName.service && sudo systemctl status $ServiceName.service --no-pager"

if ($NoRun) {
    Write-Host "Robot was deployed but is not running."
    Write-Host "Dashboard URL after starting: http://${HostName}:8080"
} elseif ($Service) {
    Invoke-Checked ssh @($sshArgs + @($remote, "$stopOldCameraCommand; $restartServiceCommand"))
    Invoke-Checked ssh @($sshArgs + @($remote, $statusServiceCommand))
    Write-Host "Service restarted. Dashboard URL: http://${HostName}:8080"
} elseif ($Run) {
    Invoke-Checked ssh @($sshArgs + @($remote, "$stopOldCameraCommand; $prepareScriptsCommand; cd '$RemoteDir' && ./build/$Target"))
} else {
    $startCommand = "$stopOldCameraCommand; if [ -f '$RemoteDir/scripts/$ServiceName.service' ]; then $restartServiceCommand; else $prepareScriptsCommand; cd '$RemoteDir' && ./build/$Target; fi"
    Invoke-Checked ssh @($sshArgs + @($remote, $startCommand))
    Invoke-Checked ssh @($sshArgs + @($remote, $statusServiceCommand))
    Write-Host "Dashboard URL: http://${HostName}:8080"
}
