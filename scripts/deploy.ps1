$ErrorActionPreference = "Stop"

$deployScript = Join-Path (Split-Path -Parent $PSScriptRoot) "deployment\deploy.ps1"
& $deployScript @args

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
