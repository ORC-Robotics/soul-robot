$ErrorActionPreference = "Stop"

$installScript = Join-Path (Split-Path -Parent $PSScriptRoot) "deployment\install-service.ps1"
& $installScript @args

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
