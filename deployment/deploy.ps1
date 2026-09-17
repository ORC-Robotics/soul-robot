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

if (-not (Test-Path -LiteralPath $KeyPath -PathType Leaf)) {
    Initialize-DeployAccess -SetupScriptPath $setupScriptPath -RaspberryHost $HostName -RaspberryUser $User -RobotServiceName $ServiceName -PrivateKeyPath $KeyPath
}

# Depois da preparação inicial, o deploy deve ser totalmente não interativo.
# O modo BatchMode impede que uma falha de acesso gere vários pedidos de senha.
$sshArgs = @("-i", $KeyPath, "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new")

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

# --- INÍCIO DA TRANSFERÊNCIA INCREMENTAL (RSYNC) ---
$rsyncSshCmd = "ssh -i `"$KeyPath`" -o IdentitiesOnly=yes -o BatchMode=yes -o StrictHostKeyChecking=accept-new"
$foldersToSync = @("CMakeLists.txt", "src", "include", "scripts", "assets")

if ($FullSync) {
    Write-Host "Sincronizando todos os arquivos de deploy e recompilando do zero..."
} else {
    Write-Host "Sincronizando arquivos de forma incremental com rsync..."
}
Push-Location $workspace
try {
    foreach ($item in $foldersToSync) {
        $rsyncArguments = @("-avz")
        if ($FullSync) {
            # Ignora data e tamanho para substituir conteúdo deixado por outro checkout.
            $rsyncArguments += "--ignore-times"
        }
        $rsyncArguments += @(
            "-e", $rsyncSshCmd,
            $item,
            "${remote}:$RemoteDir/"
        )
        Invoke-Checked -Command rsync -Arguments $rsyncArguments
    }
} catch {
    throw "Falha ao executar o rsync. Certifique-se de que o rsync está instalado e adicionado ao PATH do Windows (ex: através do Git for Windows). Erro: $_"
} finally {
    Pop-Location
}
# --- FIM DA TRANSFERÊNCIA INCREMENTAL ---

# O modo completo remove somente o build temporário. Código, dataset, ambiente
# virtual e o último binário válido permanecem intactos até o novo build passar.
if ($FullSync) {
    Invoke-Checked ssh @($sshArgs + @($remote, "rm -rf '$remoteStagingBuild'"))
}

# O deploy copia apenas os arquivos necessários para executar o robô. Os testes
# continuam ativos no build local, mas não podem exigir a pasta tests na Raspberry.
# NOTA: Adicionado '--parallel 4' no cmake --build para acelerar a compilação na Pi 5.
$atomicBuildCommand = "cd '$RemoteDir' && test -s assets/dashboard-logo.png && test -s assets/soul-sync-favicon.png && test -s assets/models/ball_detector.onnx && find scripts -type d -name '__pycache__' -prune -exec rm -rf {} + && cmake -S . -B '$remoteStagingBuild' -DBUILD_TESTING=OFF && cmake --build '$remoteStagingBuild' --target '$Target' --parallel 4 && test -s '$remoteStagingBuild/$Target' && install -m 755 '$remoteStagingBuild/$Target' '$remoteBuild/$Target.new' && mv -f '$remoteBuild/$Target.new' '$remoteBuild/$Target' && test -s '$remoteBuild/$Target'"
Invoke-Checked ssh @($sshArgs + @($remote, $atomicBuildCommand))

# Usa exatamente o mesmo Python escolhido por run_robot.sh. O ambiente virtual
# existente não é recriado nem reconfigurado por esta validação.
$visionDependenciesCommand = "if [ -x '$RemoteDir/.venv/bin/python3' ]; then python_bin='$RemoteDir/.venv/bin/python3'; else python_bin=`$(command -v python3); fi; test -n `"`$python_bin`" || { echo 'Python 3 was not found for the forward camera.' >&2; exit 1; }; if ! `"`$python_bin`" -c 'import onnxruntime; raise SystemExit(0 if tuple(map(int, onnxruntime.__version__.split(chr(46)))) == (1, 30, 0) else 1)' 2>/dev/null; then `"`$python_bin`" -m pip install --disable-pip-version-check onnxruntime==1.30.0; fi; `"`$python_bin`" -c 'import onnxruntime; raise SystemExit(0 if tuple(map(int, onnxruntime.__version__.split(chr(46)))) == (1, 30, 0) else 1)'"
Invoke-Checked ssh @($sshArgs + @($remote, $visionDependenciesCommand))

Write-Host "Deploy complete: ${remote}:$remoteBuild/$Target"

$stopOldCameraCommand = "pkill -f '$remoteCameraPattern' >/dev/null 2>&1 || true; pkill -f '$remoteForwardCameraPattern' >/dev/null 2>&1 || true"
$prepareScriptsCommand = "cd '$RemoteDir' && find scripts -type f \( -name '*.sh' -o -name '*.service' \) -exec sed -i 's/\r$//' {} + && chmod +x '$remoteRunScript' '$remoteLineCameraRunScript'"
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
