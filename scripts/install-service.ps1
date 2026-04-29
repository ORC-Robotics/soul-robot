param(
    [string]$HostName = "raspberrypi.local",
    [string]$User = "obr",
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
}

Write-Host "Installing $ServiceName service on $remote"
Invoke-Checked scp @($scpArgs + @($serviceFile, "${remote}:/tmp/$ServiceName.service"))
Invoke-Checked ssh @(
    $sshArgs +
    $remote,
    "sudo mv /tmp/$ServiceName.service /etc/systemd/system/$ServiceName.service && sudo systemctl daemon-reload && sudo systemctl enable --now $ServiceName.service && sudo systemctl status $ServiceName.service --no-pager"
)

Write-Host "Service installed. Dashboard should start automatically on boot."
