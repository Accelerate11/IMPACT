[CmdletBinding()]
param(
    [ValidatePattern('^[A-Za-z0-9._-]+$')]
    [string]$DistroName = 'Ubuntu-22.04',

    [ValidatePattern('^[A-Za-z0-9._-]+$')]
    [string]$WslUser = 'accelerate'
)

$ErrorActionPreference = 'Stop'
$sourceRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).ProviderPath
$destinationRoot = "\\wsl.localhost\$DistroName\home\$WslUser\xuanqiong_x1_sim_ws"

if (-not (Test-Path -LiteralPath (Join-Path $sourceRoot 'src\xq_sim_bringup\package.xml'))) {
    throw "Source is not the XQ isolated workspace: $sourceRoot"
}

New-Item -ItemType Directory -Path $destinationRoot -Force | Out-Null
$resolvedDestination = (Resolve-Path -LiteralPath $destinationRoot).ProviderPath
$expectedDestination = "\\wsl.localhost\$DistroName\home\$WslUser\xuanqiong_x1_sim_ws"
if ($resolvedDestination.TrimEnd('\') -ne $expectedDestination.TrimEnd('\')) {
    throw "Refusing unexpected WSL destination: $resolvedDestination"
}

function Test-IgnoredGeneratedPath {
    param([Parameter(Mandatory)][string]$RelativePath)
    $normalized = $RelativePath.Replace('/', '\')
    return $normalized -match '(^|\\)(__pycache__|\.pytest_cache)(\\|$)' -or
        $normalized -match '\.py[co]$'
}

foreach ($directoryName in @('src', 'scripts', 'docs', 'config')) {
    $sourceDirectory = Join-Path $sourceRoot $directoryName
    $destinationDirectory = Join-Path $destinationRoot $directoryName
    New-Item -ItemType Directory -Path $destinationDirectory -Force | Out-Null

    $sourceRelativePaths = [System.Collections.Generic.HashSet[string]]::new(
        [System.StringComparer]::Ordinal
    )
    Get-ChildItem -LiteralPath $sourceDirectory -Recurse -File -Force | ForEach-Object {
        $relativePath = $_.FullName.Substring($sourceDirectory.Length).TrimStart('\')
        if (Test-IgnoredGeneratedPath -RelativePath $relativePath) {
            return
        }
        [void]$sourceRelativePaths.Add($relativePath)
        $destinationFile = Join-Path $destinationDirectory $relativePath
        $destinationParent = Split-Path -Parent $destinationFile
        New-Item -ItemType Directory -Path $destinationParent -Force | Out-Null
        Copy-Item -LiteralPath $_.FullName -Destination $destinationFile -Force
    }

    # Remove only stale, non-generated files from this project's fixed source
    # subtree.  Build/install/runs and every pre-existing project are outside
    # this resolved destination and can never be selected here.
    Get-ChildItem -LiteralPath $destinationDirectory -Recurse -File -Force |
        ForEach-Object {
            $relativePath = $_.FullName.Substring($destinationDirectory.Length).TrimStart('\')
            if (Test-IgnoredGeneratedPath -RelativePath $relativePath) {
                return
            }
            if (-not $sourceRelativePaths.Contains($relativePath)) {
                $resolvedFile = $_.FullName
                $requiredPrefix = $resolvedDestination.TrimEnd('\') + '\'
                if (-not $resolvedFile.StartsWith(
                    $requiredPrefix,
                    [System.StringComparison]::OrdinalIgnoreCase
                )) {
                    throw "Refusing stale-file cleanup outside XQ destination: $resolvedFile"
                }
                Remove-Item -LiteralPath $resolvedFile -Force
            }
        }
}

foreach ($fileName in @('README.md', 'SPEC_TRACEABILITY.md', 'SOURCE_SPEC_SHA256')) {
    $sourceFile = Join-Path $sourceRoot $fileName
    if (Test-Path -LiteralPath $sourceFile) {
        Copy-Item -LiteralPath $sourceFile -Destination $destinationRoot -Force
    }
}

Write-Host "Mirrored controlled source files only to $resolvedDestination"
Write-Host 'No existing WSL project, world, or model directory was touched.'
