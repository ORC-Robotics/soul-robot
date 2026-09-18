param(
    [string]$HostName = "obr.local",
    [string]$User = "raspberry",
    [string]$RemoteDir = "/home/raspberry/OBR2026K",
    [string]$Target = "robot_test",
    [string]$KeyPath = "$env:USERPROFILE\.ssh\obr_raspberry",
    [string]$ServiceName = "obr-robot",
    [switch]$Service,
    [switch]$Run,
    [switch]$NoRun,
    [switch]$FullSync
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

function Initialize-DeployAccess {
    param(
        [string]$SetupScriptPath,
        [string]$RaspberryHost,
        [string]$RaspberryUser,
        [string]$RobotServiceName,
        [string]$PrivateKeyPath
    )

    # A preparação é necessária apenas no primeiro deploy de cada computador.
    # Ela solicita as senhas interativamente e libera somente os serviços do robô.
    Write-Host "Preparing SSH and limited sudo access for the first deploy."
    Invoke-Checked powershell.exe @(
        "-NoProfile",
        "-ExecutionPolicy", "Bypass",
        "-File", $SetupScriptPath,
        "-HostName", $RaspberryHost,
        "-User", $RaspberryUser,
        "-ServiceName", $RobotServiceName,
        "-KeyPath", $PrivateKeyPath
    )
}

# Aceita somente arquivos das árvores gerenciadas, sem caminhos de escape.
function Test-DeployPath {
    param([string]$Path)
    $parts = $Path.Split('/')
    return (($Path -ceq 'CMakeLists.txt' -or ($parts.Count -gt 1 -and $parts[0] -cin @('src', 'include', 'scripts', 'assets'))) -and
        $Path -notmatch '[\\\x00-\x1f]' -and
        @($parts | Where-Object { $_ -in @('', '.', '..', 'build', '.build-staging') -or $_ -like '.venv*' }).Count -eq 0)
}

# Substitui o JSON local atomicamente; uma interrupção não deixa um estado parcial.
function Save-DeployJson {
    param([string]$Path, $Value)
    $temporaryPath = "$Path.$([guid]::NewGuid().ToString('N')).tmp"
    try {
        [IO.File]::WriteAllText($temporaryPath, (ConvertTo-Json -InputObject $Value -Depth 5), [Text.UTF8Encoding]::new($false))
        if (Test-Path -LiteralPath $Path) {
            [IO.File]::Replace($temporaryPath, $Path, [NullString]::Value, $true)
        } else {
            [IO.File]::Move($temporaryPath, $Path)
        }
    } finally {
        if (Test-Path -LiteralPath $temporaryPath) { [IO.File]::Delete($temporaryPath) }
    }
}

$workspace = Split-Path -Parent $PSScriptRoot
$remote = "$User@$HostName"
$remoteBuild = "$RemoteDir/build"
$remoteStagingBuild = "$RemoteDir/.build-staging"
$remoteCameraPattern = "$RemoteDir/scripts/[c]amera_line_frame.py"
$remoteForwardCameraPattern = "$RemoteDir/scripts/[f]orward_camera_stream.py"
$remoteRunScript = "$RemoteDir/scripts/run_robot.sh"
$remoteLineCameraRunScript = "$RemoteDir/scripts/run_line_camera.sh"
$lineCameraServiceName = "obr-line-camera"
$setupScriptPath = Join-Path $PSScriptRoot "install-service.ps1"
$setupCommand = "powershell -ExecutionPolicy Bypass -File deployment/install-service.ps1 -HostName $HostName"

# As imagens do dashboard são lidas em tempo de execução. Validá-las antes de
# parar o serviço evita deixar o robô indisponível por causa de um pacote incompleto.
$requiredDashboardAssets = @(
    "$workspace/assets/dashboard-logo.png",
    "$workspace/assets/soul-sync-favicon.png",
    # O processo da câmera não deve iniciar sem o modelo de vítimas validado.
    "$workspace/assets/models/ball_detector.onnx"
)

foreach ($assetPath in $requiredDashboardAssets) {
    if (-not (Test-Path -LiteralPath $assetPath -PathType Leaf)) {
        throw "Dashboard asset not found: $assetPath"
    }
}

# O estado pertence ao notebook e ao destino, incluindo a pasta local do projeto.
# Não depende de commits nem de uma varredura remota a cada deploy.
$stateIdentity = "$workspace`n$remote`n$($RemoteDir.TrimEnd('/'))"
$sha256 = [Security.Cryptography.SHA256]::Create()
try {
    $stateKey = [BitConverter]::ToString($sha256.ComputeHash([Text.Encoding]::UTF8.GetBytes($stateIdentity))).Replace('-', '').ToLowerInvariant()
} finally { $sha256.Dispose() }
$stateDirectory = Join-Path $env:LOCALAPPDATA "OBR2026K/deploy/$stateKey"
[IO.Directory]::CreateDirectory($stateDirectory) | Out-Null
$manifestPath = Join-Path $stateDirectory 'manifest.json'
$pendingPath = Join-Path $stateDirectory 'pending.json'

# Impede que dois deploys do mesmo destino confirmem estados diferentes ao mesmo tempo.
$stateLock = [IO.File]::Open((Join-Path $stateDirectory 'deploy.lock'), 'OpenOrCreate', 'ReadWrite', 'None')
try {
$previousFiles = [Collections.Generic.Dictionary[string, string]]::new([StringComparer]::Ordinal)
if (Test-Path -LiteralPath $manifestPath) {
    try {
        $manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
        if ($manifest.version -ne 1 -or $null -eq $manifest.files) { throw 'Invalid manifest format.' }
        foreach ($property in $manifest.files.PSObject.Properties) {
            if (-not (Test-DeployPath $property.Name) -or $property.Value -notmatch '^[0-9A-Fa-f]{64}$') { throw 'Invalid manifest entry.' }
            $previousFiles.Add($property.Name, $property.Value)
        }
    } catch {
        if (-not $FullSync) { throw "Cannot read deploy manifest. Retry with -FullSync: $_" }
        $previousFiles.Clear()
        Write-Warning 'Manifesto inválido ignorado; não é possível identificar remoções do estado perdido.'
    }
}
$pendingFiles = [Collections.Generic.HashSet[string]]::new([StringComparer]::Ordinal)
if (Test-Path -LiteralPath $pendingPath) {
    foreach ($path in (Get-Content -LiteralPath $pendingPath -Raw -Encoding UTF8 | ConvertFrom-Json)) {
        if (-not (Test-DeployPath $path)) { throw "Invalid pending deploy path: $path" }
        [void]$pendingFiles.Add($path)
    }
}

$currentFiles = [Collections.Generic.Dictionary[string, string]]::new([StringComparer]::Ordinal)
$localFiles = @((Get-Item -LiteralPath "$workspace/CMakeLists.txt"))
# Não atravessa links ou junctions que poderiam incluir datasets e pastas externas.
$directories = [Collections.Generic.Stack[string]]::new()
foreach ($tree in @('src', 'include', 'scripts', 'assets')) { $directories.Push((Join-Path $workspace $tree)) }
while ($directories.Count -gt 0) {
    $directory = Get-Item -LiteralPath $directories.Pop() -Force
    if ($directory.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Deploy does not support links: $($directory.FullName)" }
    foreach ($item in (Get-ChildItem -LiteralPath $directory.FullName -Force)) {
        $relativePath = $item.FullName.Substring($workspace.Length + 1).Replace('\', '/')
        if (-not (Test-DeployPath $relativePath)) { throw "Unsafe deploy path: $relativePath" }
        if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Deploy does not support links: $($item.FullName)" }
        if ($item.PSIsContainer) { $directories.Push($item.FullName) } else { $localFiles += $item }
    }
}
foreach ($file in $localFiles) {
    if ($file.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Deploy does not support links: $($file.FullName)" }
    $path = $file.FullName.Substring($workspace.Length + 1).Replace('\', '/')
    $currentFiles.Add($path, (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash)
}
$changedFiles = @($currentFiles.Keys | Where-Object {
    $FullSync -or -not $previousFiles.ContainsKey($_) -or $currentFiles[$_] -ne $previousFiles[$_] -or $pendingFiles.Contains($_)
} | Sort-Object)
$knownFiles = [Collections.Generic.HashSet[string]]::new([StringComparer]::Ordinal)
foreach ($path in $previousFiles.Keys) { [void]$knownFiles.Add($path) }
foreach ($path in $pendingFiles) { [void]$knownFiles.Add($path) }
$removedFiles = @($knownFiles | Where-Object { -not $currentFiles.ContainsKey($_) } | Sort-Object)
Write-Host "[SYNC] $($changedFiles.Count) arquivos alterados"
foreach ($path in $changedFiles) { Write-Host $path }
if ($removedFiles.Count -gt 0) {
    Write-Host "[SYNC] $($removedFiles.Count) arquivos removidos"
    foreach ($path in $removedFiles) { Write-Host $path }
}

# O diário não confirma o deploy. Guarda tentativas para repetir uploads após falhas,
# inclusive quando um arquivo é revertido ou removido antes da próxima tentativa.
foreach ($path in $changedFiles) { [void]$pendingFiles.Add($path) }
foreach ($path in $removedFiles) { [void]$pendingFiles.Add($path) }
Save-DeployJson $pendingPath @($pendingFiles)

$archivePath = Join-Path $stateDirectory "$([guid]::NewGuid().ToString('N')).zip"
$remoteArchive = "/tmp/obr-deploy-$([guid]::NewGuid().ToString('N')).zip"
try {
if ((-not $FullSync -and $changedFiles.Count -gt 0) -or $removedFiles.Count -gt 0) {
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $archive = [IO.Compression.ZipFile]::Open($archivePath, 'Create')
    try {
        if (-not $FullSync) {
            foreach ($path in $changedFiles) {
                # Mantém o arquivo aberto sem permitir gravações enquanto confere e empacota.
                $source = [IO.File]::Open((Join-Path $workspace $path), 'Open', 'Read', 'Read')
                try {
                    $sha256 = [Security.Cryptography.SHA256]::Create()
                    try { $hash = [BitConverter]::ToString($sha256.ComputeHash($source)).Replace('-', '') } finally { $sha256.Dispose() }
                    if ($hash -ne $currentFiles[$path]) { throw "File changed during packaging; retry deploy: $path" }
                    $source.Position = 0
                    $entry = $archive.CreateEntry($path, [IO.Compression.CompressionLevel]::Fastest)
                    $output = $entry.Open()
                    try { $source.CopyTo($output) } finally { $output.Dispose() }
                } finally { $source.Dispose() }
            }
        }
        $entry = $archive.CreateEntry('_deploy_removed.json')
        $writer = [IO.StreamWriter]::new($entry.Open(), [Text.UTF8Encoding]::new($false))
        try { $writer.Write((ConvertTo-Json -InputObject $removedFiles)) } finally { $writer.Dispose() }
    } finally { $archive.Dispose() }
}

if (-not (Test-Path -LiteralPath $KeyPath -PathType Leaf)) {
    Initialize-DeployAccess -SetupScriptPath $setupScriptPath -RaspberryHost $HostName -RaspberryUser $User -RobotServiceName $ServiceName -PrivateKeyPath $KeyPath
}

# Depois da preparação inicial, o deploy deve ser totalmente não interativo.
# O modo BatchMode impede que uma falha de acesso gere vários pedidos de senha.
$sshArgs = @("-i", $KeyPath, "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new")
$scpArgs = @("-i", $KeyPath, "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new")

& ssh @($sshArgs + @($remote, "true"))
if ($LASTEXITCODE -ne 0) {
    Initialize-DeployAccess -SetupScriptPath $setupScriptPath -RaspberryHost $HostName -RaspberryUser $User -RobotServiceName $ServiceName -PrivateKeyPath $KeyPath

    & ssh @($sshArgs + @($remote, "true"))
    if ($LASTEXITCODE -ne 0) {
        throw "SSH key access could not be configured for $remote. Retry manually with: $setupCommand"
    }
}

Write-Host "Deploying to ${remote}:$RemoteDir"

Invoke-Checked ssh @($sshArgs + @($remote, "mkdir -p '$RemoteDir' '$remoteBuild'"))

# Para o serviço antes de trocar código ou binário. Se o build falhar, o robô
# permanece parado e o executável válido anterior não é substituído.
Invoke-Checked ssh @($sshArgs + @($remote, "sudo -n systemctl stop '$ServiceName.service' '$lineCameraServiceName.service' || { echo 'Deploy access is not configured. Run: $setupCommand' >&2; exit 1; }"))
if (Test-Path -LiteralPath $archivePath) {
    Write-Host '[SYNC] enviando alterações...'
    try {
    Invoke-Checked scp @($scpArgs + @($archivePath, "${remote}:$remoteArchive"))
    # Usa somente a biblioteca padrão do Python 3 já utilizado pela visão.
    # Valida todos os caminhos antes de excluir arquivos ou extrair o ZIP.
    $applyChanges = @'
import base64, json, pathlib, shutil, zipfile
settings = json.loads(base64.b64decode('SETTINGS_BASE64'))
root = pathlib.Path(settings['root']).resolve(strict=True)
def checked_path(name):
    parts = name.split('/')
    if not (name == 'CMakeLists.txt' or (len(parts) > 1 and parts[0] in ('src', 'include', 'scripts', 'assets'))):
        raise ValueError('Unmanaged deploy path: ' + name)
    if '\\' in name or any(ord(c) < 32 for c in name) or any(p in ('', '.', '..', 'build', '.build-staging') or p.startswith('.venv') for p in parts):
        raise ValueError('Unsafe deploy path: ' + name)
    target = root.joinpath(*parts)
    for parent in (target, *target.parents):
        if parent == root:
            break
        if parent.is_symlink():
            raise ValueError('Deploy path contains a symlink: ' + name)
    return target
with zipfile.ZipFile(settings['archive']) as archive:
    removed = json.loads(archive.read('_deploy_removed.json'))
    entries = [e for e in archive.infolist() if e.filename != '_deploy_removed.json']
    targets = [(e, checked_path(e.filename)) for e in entries]
    deletions = [checked_path(name) for name in removed]
    for target in deletions:
        if target.exists():
            target.unlink()
        parent = target.parent
        while parent.parent != root and parent != root:
            try:
                parent.rmdir()
            except OSError:
                break
            parent = parent.parent
    for entry, target in targets:
        target.parent.mkdir(parents=True, exist_ok=True)
        with archive.open(entry) as source, target.open('wb') as output:
            shutil.copyfileobj(source, output)
'@
    $settings = ConvertTo-Json -Compress @{ root = $RemoteDir; archive = $remoteArchive }
    $settingsBase64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($settings))
    $applyChanges = $applyChanges.Replace('SETTINGS_BASE64', $settingsBase64)
    # Envia o script pelo stdin para evitar que PowerShell e SSH removam aspas
    # do argumento python3 -c no Windows.
    $applyChanges | & ssh @($sshArgs + @($remote, "python3 -"))
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to apply deploy archive on $remote"
    }
    } finally {
        Invoke-Checked ssh @($sshArgs + @($remote, "rm -f -- '$remoteArchive'"))
    }
} elseif (-not $FullSync) {
    Write-Host '[SYNC] nada para enviar'
}
if ($FullSync) {
Write-Host '[SYNC] sincronização completa via SCP...'
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
Invoke-Checked scp @(
    $scpArgs +
    "-r",
    "$workspace/assets",
    "${remote}:$RemoteDir/"
)
foreach ($file in $localFiles) {
    $path = $file.FullName.Substring($workspace.Length + 1).Replace('\', '/')
    if ((Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash -ne $currentFiles[$path]) {
        throw "File changed during full upload; retry deploy: $path"
    }
}
}

# O deploy copia apenas os arquivos necessários para executar o robô. Os testes
# continuam ativos no build local, mas não podem exigir a pasta tests na Raspberry.
$atomicBuildCommand = "cd '$RemoteDir' && test -s assets/dashboard-logo.png && test -s assets/soul-sync-favicon.png && test -s assets/models/ball_detector.onnx && find scripts -type d -name '__pycache__' -prune -exec rm -rf {} + && cmake -S . -B '$remoteStagingBuild' -DBUILD_TESTING=OFF && cmake --build '$remoteStagingBuild' --target '$Target' && test -s '$remoteStagingBuild/$Target' && install -m 755 '$remoteStagingBuild/$Target' '$remoteBuild/$Target.new' && mv -f '$remoteBuild/$Target.new' '$remoteBuild/$Target' && test -s '$remoteBuild/$Target'"
Write-Host '[BUILD] iniciando build remoto...'
Invoke-Checked ssh @($sshArgs + @($remote, $atomicBuildCommand))

# Usa exatamente o mesmo Python escolhido por run_robot.sh. O ambiente virtual
# existente não é recriado nem reconfigurado por esta validação.
$visionDependenciesCommand = "if [ -x '$RemoteDir/.venv/bin/python3' ]; then python_bin='$RemoteDir/.venv/bin/python3'; else python_bin=`$(command -v python3); fi; test -n `"`$python_bin`" || { echo 'Python 3 was not found for the forward camera.' >&2; exit 1; }; if ! `"`$python_bin`" -c 'import onnxruntime; raise SystemExit(0 if tuple(map(int, onnxruntime.__version__.split(chr(46)))) == (1, 30, 0) else 1)' 2>/dev/null; then `"`$python_bin`" -m pip install --disable-pip-version-check onnxruntime==1.30.0; fi; `"`$python_bin`" -c 'import onnxruntime; raise SystemExit(0 if tuple(map(int, onnxruntime.__version__.split(chr(46)))) == (1, 30, 0) else 1)'"
Invoke-Checked ssh @($sshArgs + @($remote, $visionDependenciesCommand))

$stopOldCameraCommand = "pkill -f '$remoteCameraPattern' >/dev/null 2>&1 || true; pkill -f '$remoteForwardCameraPattern' >/dev/null 2>&1 || true"
$prepareScriptsCommand = "cd '$RemoteDir' && find scripts -type f \( -name '*.sh' -o -name '*.service' \) -exec sed -i 's/\r$//' {} + && chmod +x '$remoteRunScript' '$remoteLineCameraRunScript'"
if (-not $FullSync) {
    # sed -i regrava até arquivos sem CRLF; só executa quando há algo a converter.
    $prepareScriptsCommand = "cd '$RemoteDir' && find scripts -type f \( -name '*.sh' -o -name '*.service' \) -exec sh -c 'for file do if LC_ALL=C grep -q `"`$(printf `"\r`" )`$`" `"`$file`"; then sed -i `"s/\r`$//`" `"`$file`" || exit 1; fi; done' sh {} + && chmod +x '$remoteRunScript' '$remoteLineCameraRunScript'"
}
$restartServiceCommand = "$prepareScriptsCommand && sudo -n systemctl restart '$ServiceName.service' '$lineCameraServiceName.service'"
$statusServiceCommand = "sudo -n systemctl is-active --quiet '$ServiceName.service' && sudo -n systemctl is-active --quiet '$lineCameraServiceName.service' && sudo -n systemctl status '$ServiceName.service' '$lineCameraServiceName.service' --no-pager"

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
    Invoke-Checked ssh @($sshArgs + @($remote, "$stopOldCameraCommand; $restartServiceCommand"))
    Invoke-Checked ssh @($sshArgs + @($remote, $statusServiceCommand))
    Write-Host "Dashboard URL: http://${HostName}:8080"
}

# Confirma somente depois do upload, build, instalação e validações do modo escolhido.
Save-DeployJson $manifestPath @{ version = 1; files = $currentFiles }
[IO.File]::Delete($pendingPath)
Write-Host "[DEPLOY] concluído: ${remote}:$remoteBuild/$Target"
} finally {
    if (Test-Path -LiteralPath $archivePath) { [IO.File]::Delete($archivePath) }
}
} finally { $stateLock.Dispose() }
