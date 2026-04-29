param(
    [string]$HostName = "192.168.0.104",
    [string]$User = "obr",
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

$sshDir = Split-Path -Parent $KeyPath
$remote = "$User@$HostName"

if (-not (Test-Path $sshDir)) {
    New-Item -ItemType Directory -Path $sshDir | Out-Null
}

if (-not (Test-Path $KeyPath)) {
    Invoke-Expression "ssh-keygen --% -t ed25519 -f $KeyPath -N `"`" -C obr-deploy"
    if (-not (Test-Path $KeyPath)) {
        throw "Command failed: ssh-keygen did not create $KeyPath"
    }
}

Write-Host "Sending public key to $remote"
Write-Host "Type the Raspberry password once when prompted."

$publicKey = & ssh-keygen -y -f $KeyPath
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($publicKey)) {
    throw "Command failed: ssh-keygen could not read public key from $KeyPath"
}

$installCommand = 'mkdir -p ~/.ssh && touch ~/.ssh/authorized_keys && cat >> ~/.ssh/authorized_keys && awk ''!seen[$0]++'' ~/.ssh/authorized_keys > ~/.ssh/authorized_keys.tmp && mv ~/.ssh/authorized_keys.tmp ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys'
$publicKey | & ssh $remote $installCommand
if ($LASTEXITCODE -ne 0) {
    throw "Command failed: ssh $remote install public key"
}

Write-Host "SSH key configured. Future deploys should not ask for the Raspberry password."
