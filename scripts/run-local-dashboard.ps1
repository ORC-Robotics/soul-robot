$ErrorActionPreference = "Stop"

$dashboardScript = Join-Path (Split-Path -Parent $PSScriptRoot) "tools\run-local-dashboard.ps1"
& $dashboardScript @args

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
