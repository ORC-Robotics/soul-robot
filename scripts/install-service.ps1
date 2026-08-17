param(
    [string]$HostName = "192.168.0.102",
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
$remote = "$User@$HostName"
$sshArgs = @()
$scpArgs = @()

if (Test-Path $KeyPath) {
    $sshArgs += @("-i", $KeyPath)
    $scpArgs += @("-i", $KeyPath)
} else {
    Write-Host "SSH key not found at $KeyPath. SSH may ask for the Raspberry password."
}

Write-Host "Installing $ServiceName service on $remote"
Invoke-Checked scp @($scpArgs + @($serviceFile, "${remote}:/tmp/$ServiceName.service"))
Invoke-Checked ssh @(
    $sshArgs +
    $remote,
    "sudo mv /tmp/$ServiceName.service /etc/systemd/system/$ServiceName.service && sudo systemctl daemon-reload && sudo systemctl enable --now $ServiceName.service && sudo systemctl status $ServiceName.service --no-pager"
)

Write-Host "Service installed. Dashboard should start automatically on boot."
