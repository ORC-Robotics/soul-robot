param(
    [string]$HostName = "obr.local",
    [string]$User = "raspberry",
    [string]$RemoteDir = "/home/raspberry/OBR2026K",
    [string]$KeyPath = "$env:USERPROFILE\.ssh\obr_raspberry",
    [ValidateSet("forward", "down")]
    [string]$Camera,
    [ValidatePattern("^[A-Za-z0-9_-]+$")]
    [string]$Session,
    [double]$IntervalSeconds = 2.0,
    [switch]$Once
)

$ErrorActionPreference = "Stop"

if ($IntervalSeconds -lt 0.5) {
    throw "IntervalSeconds deve ser igual ou superior a 0,5 segundo."
}
if ($Session -and -not $Camera) {
    throw "Informe -Camera forward ou -Camera down ao selecionar uma sessão."
}

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$localRoot = Join-Path $projectRoot "yolo_ball\datasets\images\raw"
$remoteRoot = "$RemoteDir/yolo_ball/datasets/images/raw"
$remoteSearchRoot = $remoteRoot
$relativePrefix = ""
if ($Camera) {
    $remoteSearchRoot = "$remoteSearchRoot/$Camera"
    $relativePrefix = "$Camera/"
}
if ($Session) {
    $remoteSearchRoot = "$remoteSearchRoot/$Session"
    $relativePrefix = "$relativePrefix$Session/"
}
$remote = "$User@$HostName"
$sshArgs = @("-i", $KeyPath, "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5")
$scpArgs = @("-i", $KeyPath, "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5")

if (-not (Test-Path -LiteralPath $KeyPath -PathType Leaf)) {
    throw "Chave SSH não encontrada: $KeyPath. Execute o deploy uma vez para configurá-la."
}

New-Item -ItemType Directory -Force -Path $localRoot | Out-Null
Write-Host "Sincronização YOLO ativa. Pressione Ctrl+C para encerrar."
Write-Host "Raspberry: ${remote}:$remoteSearchRoot"
Write-Host "Computador: $localRoot"

do {
    $remoteListPath = $null
    $remoteArchivePath = $null
    $temporaryRoot = $null

    try {
        # Os nomes são gerados pelo recorder e contêm apenas diretórios validados
        # e timestamps numéricos, evitando interpretar caminhos arbitrários.
        $listedPaths = & ssh @sshArgs $remote "find '$remoteSearchRoot' -type f -name '*.jpg' -printf '%P\n' 2>/dev/null || true"
        if ($LASTEXITCODE -ne 0) {
            throw "Não foi possível consultar a Raspberry Pi."
        }

        $missingPaths = [System.Collections.Generic.List[string]]::new()
        foreach ($listedPath in @($listedPaths)) {
            $relativePath = "$relativePrefix$($listedPath.Trim())"
            if (-not $relativePath -or
                $relativePath -notmatch '^[A-Za-z0-9_-]+(?:/[A-Za-z0-9_.-]+)*\.jpg$' -or
                $relativePath.Contains("..")) {
                continue
            }

            $localPath = Join-Path $localRoot ($relativePath.Replace('/', '\'))
            $pathParts = $relativePath.Split('/')
            if ($pathParts.Count -ge 4) {
                $localSession = Join-Path $localRoot (Join-Path $pathParts[0] $pathParts[1])
                # Pastas vazias não são transferidas pelo SCP. Criá-las aqui
                # mantém a separação visível mesmo antes da primeira amostra.
                foreach ($className in @("black", "silver", "other")) {
                    New-Item -ItemType Directory -Force -Path (Join-Path $localSession $className) | Out-Null
                }
            }
            if (Test-Path -LiteralPath $localPath -PathType Leaf) {
                continue
            }

            $missingPaths.Add($relativePath)
        }

        if ($missingPaths.Count -eq 0) {
            Write-Host "Nenhuma imagem nova."
        }
        else {
            # Um único arquivo TAR evita abrir uma conexão SCP para cada JPEG.
            # JPEG já é comprimido, portanto compactar novamente apenas gastaria CPU.
            $syncToken = [Guid]::NewGuid().ToString("N")
            $temporaryBase = [IO.Path]::GetFullPath([IO.Path]::GetTempPath())
            $temporaryRoot = [IO.Path]::GetFullPath(
                (Join-Path $temporaryBase "obr-yolo-sync-$syncToken")
            )
            if (-not $temporaryRoot.StartsWith($temporaryBase, [StringComparison]::OrdinalIgnoreCase)) {
                throw "Diretório temporário fora da pasta esperada: $temporaryRoot"
            }
            New-Item -ItemType Directory -Path $temporaryRoot | Out-Null

            $localListPath = Join-Path $temporaryRoot "files.txt"
            $localArchivePath = Join-Path $temporaryRoot "images.tar"
            $remoteListPath = "/tmp/obr-yolo-sync-$syncToken.txt"
            $remoteArchivePath = "/tmp/obr-yolo-sync-$syncToken.tar"
            # O TAR roda em Linux e exige LF; WriteAllLines usaria CRLF no Windows.
            $listContent = [String]::Join("`n", $missingPaths) + "`n"
            [IO.File]::WriteAllText(
                $localListPath,
                $listContent,
                [Text.UTF8Encoding]::new($false)
            )

            & scp @scpArgs $localListPath "${remote}:$remoteListPath"
            if ($LASTEXITCODE -ne 0) {
                throw "Não foi possível enviar a lista de imagens ao Raspberry Pi."
            }

            & ssh @sshArgs $remote "tar -cf '$remoteArchivePath' -C '$remoteRoot' -T '$remoteListPath'"
            if ($LASTEXITCODE -ne 0) {
                throw "Não foi possível criar o lote de imagens no Raspberry Pi."
            }

            & scp @scpArgs "${remote}:$remoteArchivePath" $localArchivePath
            if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $localArchivePath -PathType Leaf)) {
                throw "Não foi possível transferir o lote de imagens."
            }

            & tar -xf $localArchivePath -C $localRoot
            if ($LASTEXITCODE -ne 0) {
                throw "Não foi possível extrair o lote de imagens no computador."
            }

            $receivedCount = 0
            foreach ($relativePath in $missingPaths) {
                $localPath = Join-Path $localRoot ($relativePath.Replace('/', '\'))
                if (-not (Test-Path -LiteralPath $localPath -PathType Leaf)) {
                    throw "Imagem ausente após a extração: $relativePath"
                }
                $receivedCount++
            }
            Write-Host "Recebidas em lote: $receivedCount imagens."
        }
    }
    catch {
        Write-Warning $_.Exception.Message
    }
    finally {
        if ($remoteListPath -and $remoteArchivePath) {
            & ssh @sshArgs $remote "rm -f '$remoteListPath' '$remoteArchivePath'" 2>$null
        }
        if ($temporaryRoot -and (Test-Path -LiteralPath $temporaryRoot)) {
            # O caminho absoluto foi validado dentro da pasta temporária acima.
            Remove-Item -LiteralPath $temporaryRoot -Recurse -Force
        }
    }

    if (-not $Once) {
        Start-Sleep -Milliseconds ([int]($IntervalSeconds * 1000))
    }
} while (-not $Once)
