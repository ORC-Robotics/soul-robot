$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$dashboardUrl = "http://127.0.0.1:8080"

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    throw "CMake não foi encontrado. Instale o CMake e execute este script novamente."
}

$cmakeArguments = @("-S", $projectRoot)
if (Get-Command ninja -ErrorAction SilentlyContinue) {
    $buildDirectory = Join-Path $projectRoot "build-local-ninja"
    $cmakeArguments += @("-B", $buildDirectory, "-G", "Ninja")
} else {
    $buildDirectory = Join-Path $projectRoot "build-local"
    $cmakeArguments += @("-B", $buildDirectory)
}

Write-Host "Preparando o dashboard local..."

cmake @cmakeArguments
if ($LASTEXITCODE -ne 0) {
    throw "A configuração local do CMake falhou."
}

cmake --build $buildDirectory --config Release
if ($LASTEXITCODE -ne 0) {
    throw "A compilação local falhou."
}

$executableCandidates = @(
    (Join-Path $buildDirectory "Release\robot_test.exe"),
    (Join-Path $buildDirectory "robot_test.exe")
)
$executable = $executableCandidates |
    Where-Object { Test-Path $_ } |
    Select-Object -First 1

if (-not $executable) {
    throw "O executável local não foi encontrado após a compilação."
}

Write-Host ""
Write-Host "Dashboard local: $dashboardUrl"
Write-Host "Abra esse endereço no navegador e mantenha este terminal aberto."
Write-Host "Pressione Ctrl+C para encerrar o servidor local."
Write-Host ""

# No Windows, GPIO, UART e motores usam implementações inativas de desenvolvimento.
# Isso permite visualizar o painel sem conectar a Raspberry Pi ou movimentar o robô.
& $executable

if ($LASTEXITCODE -ne 0) {
    throw "O servidor local terminou com o código $LASTEXITCODE."
}
