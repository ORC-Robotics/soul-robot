param(
    [string]$HostName = "192.168.0.104",
    [string]$User = "raspberry",
    [string]$ServiceName = "obr-robot",
    [string]$KeyPath = "$env:USERPROFILE\.ssh\obr_raspberry"
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
$serviceFile = Join-Path $workspace "scripts\$ServiceName.service"
$lineCameraServiceName = "obr-line-camera"
$lineCameraServiceFile = Join-Path $workspace "scripts\$lineCameraServiceName.service"
$remote = "$User@$HostName"
$sshArgs = @()
$scpArgs = @()

if (Test-Path $KeyPath) {
    $sshArgs += @("-i", $KeyPath)
    $scpArgs += @("-i", $KeyPath)
} else {
    Write-Host "SSH key not found at $KeyPath. SSH may ask for the Raspberry password."
}

Write-Host "Installing $ServiceName and $lineCameraServiceName services on $remote"
Invoke-Checked scp @($scpArgs + @($serviceFile, $lineCameraServiceFile, "${remote}:/tmp/"))
Invoke-Checked ssh @(
    $sshArgs +
    $remote,
    "chmod +x /home/$User/OBR2026K/scripts/run_robot.sh /home/$User/OBR2026K/scripts/run_line_camera.sh && sudo mv /tmp/$ServiceName.service /tmp/$lineCameraServiceName.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now $ServiceName.service $lineCameraServiceName.service && sudo systemctl status $ServiceName.service $lineCameraServiceName.service --no-pager"
)

Write-Host "Service installed. Dashboard should start automatically on boot."
