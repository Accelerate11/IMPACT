param(
  [string]$WslDistro = "Ubuntu-22.04",
  [string]$SourceRun = "experiments/results/impact_complex_comparison/gate_20260902T122517Z_286",
  [string]$DestinationRun = "compositional_gate_20260902T122517Z_286"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$wslRoot = "\\wsl.localhost\$WslDistro\home\accelerate\xuanqiong_x1_sim_ws"
$sourceRoot = Join-Path $wslRoot ($SourceRun -replace '/', '\')
$destinationRoot = Join-Path (Join-Path $repoRoot "evidence\COMPLEX_DEMO") $DestinationRun

if (-not (Test-Path -LiteralPath $sourceRoot -PathType Container)) {
  throw "Formal complex-compositional run not found: $sourceRoot"
}

$sourceRootResolved = (Get-Item -LiteralPath $sourceRoot).FullName.TrimEnd('\')
$destinationParent = Split-Path -Parent $destinationRoot
$destinationParentResolved = (Get-Item -LiteralPath $destinationParent).FullName.TrimEnd('\')
if (-not $destinationParentResolved.StartsWith((Join-Path $repoRoot "evidence\COMPLEX_DEMO"), [System.StringComparison]::OrdinalIgnoreCase)) {
  throw "Refusing to archive outside evidence/COMPLEX_DEMO: $destinationRoot"
}

$lightweightFiles = Get-ChildItem -LiteralPath $sourceRootResolved -File -Recurse | Where-Object {
  $_.FullName -notmatch '\\ros_logs\\' -and
  $_.Name -notmatch '\.(db3|zstd|tlog|tlog-journal)$'
}

foreach ($file in $lightweightFiles) {
  $relativePath = $file.FullName.Substring($sourceRootResolved.Length + 1)
  $destination = Join-Path $destinationRoot $relativePath
  New-Item -ItemType Directory -Force -Path (Split-Path -Parent $destination) | Out-Null
  Copy-Item -LiteralPath $file.FullName -Destination $destination -Force
}

$largeFiles = Get-ChildItem -LiteralPath $sourceRootResolved -File -Recurse | Where-Object {
  $_.Name -match '\.(db3|zstd|tlog|tlog-journal)$'
} | Sort-Object FullName

$hashManifest = @("# SHA256  BYTES  WSL_RELATIVE_PATH")
$byteManifest = @("# BYTES  WSL_RELATIVE_PATH")
foreach ($file in $largeFiles) {
  $hash = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
  $relativePath = $file.FullName.Substring($wslRoot.Length + 1).Replace('\', '/')
  $hashManifest += "$hash  $($file.Length)  $relativePath"
  $byteManifest += "$($file.Length)  $relativePath"
}

New-Item -ItemType Directory -Force -Path $destinationRoot | Out-Null
$utf8WithoutBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText(
  (Join-Path $destinationRoot "LARGE_ARTIFACTS.sha256"),
  (($hashManifest -join "`n") + "`n"),
  $utf8WithoutBom
)
[System.IO.File]::WriteAllText(
  (Join-Path $destinationRoot "LARGE_ARTIFACTS.bytes"),
  (($byteManifest -join "`n") + "`n"),
  $utf8WithoutBom
)

Write-Host "Archived $($lightweightFiles.Count) lightweight files and indexed $($largeFiles.Count) large artifacts."
Write-Host "Destination: $destinationRoot"
