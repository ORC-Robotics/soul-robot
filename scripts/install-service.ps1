param(
    [string]$HostName = "obr.local",
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

function Test-SshKeyAccess {
    param(
        [string]$Remote,
        [string]$PrivateKeyPath
    )

    # A primeira tentativa normalmente falha antes de a chave ser instalada.
    # O Windows PowerShell converte a saída de erro nativa em NativeCommandError;
    # isso não deve interromper a preparação antes do pedido de senha.
    $previousErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "SilentlyContinue"

    try {
        & ssh -i $PrivateKeyPath -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new $Remote "true" 2>$null
        $sshExitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }

    return $sshExitCode -eq 0
}

if ($User -notmatch '^[a-z_][a-z0-9_-]*$') {
    throw "Invalid Raspberry user: $User"
}

if ($ServiceName -notmatch '^[a-zA-Z0-9_.@-]+$') {
    throw "Invalid systemd service name: $ServiceName"
}

$workspace = Split-Path -Parent $PSScriptRoot
$serviceFile = Join-Path $workspace "scripts\$ServiceName.service"
$lineCameraServiceName = "obr-line-camera"
$lineCameraServiceFile = Join-Path $workspace "scripts\$lineCameraServiceName.service"
$sudoersTemplateFile = Join-Path $workspace "scripts\obr-deploy.sudoers"
$remoteInstallerFile = Join-Path $workspace "scripts\install-service-remote.sh"
$remote = "$User@$HostName"

if (-not (Test-Path -LiteralPath $KeyPath -PathType Leaf)) {
    $keyDirectory = Split-Path -Parent $KeyPath
    New-Item -ItemType Directory -Path $keyDirectory -Force | Out-Null

    Write-Host "Creating the deploy SSH key at $KeyPath"

    # O Windows PowerShell 5.1 remove argumentos vazios ao chamar programas
    # nativos. As aspas literais preservam a senha vazia esperada pelo ssh-keygen.
    $emptyPassphraseArgument = '""'
    Invoke-Checked ssh-keygen @("-t", "ed25519", "-f", $KeyPath, "-N", $emptyPassphraseArgument, "-C", "obr2026k-deploy")
}

if (-not (Test-SshKeyAccess -Remote $remote -PrivateKeyPath $KeyPath)) {
    $publicKey = (& ssh-keygen -y -f $KeyPath)
    if ($LASTEXITCODE -ne 0) {
        throw "Could not read the public key from $KeyPath"
    }

    $publicKeyParts = @($publicKey -split '\s+')
    if ($publicKeyParts.Count -lt 2 -or
        $publicKeyParts[0] -notmatch '^(ssh-(ed25519|rsa)|ecdsa-sha2-nistp(256|384|521))$' -or
        $publicKeyParts[1] -notmatch '^[A-Za-z0-9+/=]+$') {
        throw "Invalid public key generated from $KeyPath"
    }

    $publicKeyLine = "$($publicKeyParts[0]) $($publicKeyParts[1]) obr2026k-deploy"
    $installKeyCommand = "umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; grep -qF '$($publicKeyParts[0]) $($publicKeyParts[1])' ~/.ssh/authorized_keys || printf '%s\n' '$publicKeyLine' >> ~/.ssh/authorized_keys; chmod 700 ~/.ssh; chmod 600 ~/.ssh/authorized_keys"

    Write-Host "Enter the Raspberry SSH password once to install the deploy key."
    Invoke-Checked ssh @("-i", $KeyPath, "-o", "IdentitiesOnly=yes", "-o", "StrictHostKeyChecking=accept-new", $remote, $installKeyCommand)
}

if (-not (Test-SshKeyAccess -Remote $remote -PrivateKeyPath $KeyPath)) {
    throw "The SSH key was not accepted by $remote"
}

$sshArgs = @("-i", $KeyPath, "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new")
$scpArgs = @("-i", $KeyPath, "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new")

Write-Host "Installing $ServiceName and $lineCameraServiceName services on $remote"
Invoke-Checked scp @(
    $scpArgs + @(
        $serviceFile,
        $lineCameraServiceFile,
        $sudoersTemplateFile,
        $remoteInstallerFile,
        "${remote}:/tmp/"
    )
)

Write-Host "Enter the Raspberry sudo password once to finish the service setup."
$normalizeFilesCommand = "sed -i 's/\r$//' /tmp/install-service-remote.sh /tmp/obr-deploy.sudoers '/tmp/$ServiceName.service' '/tmp/$lineCameraServiceName.service'"
$remoteInstallCommand = "$normalizeFilesCommand && chmod +x /tmp/install-service-remote.sh && /tmp/install-service-remote.sh '$User' '$ServiceName' '$lineCameraServiceName'"
Invoke-Checked ssh @($sshArgs + @("-t", $remote, $remoteInstallCommand))

Write-Host "Setup complete. Future deploys will not ask for SSH or sudo passwords."
