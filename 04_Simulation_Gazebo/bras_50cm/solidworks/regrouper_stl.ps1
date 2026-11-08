$bureau = [Environment]::GetFolderPath('Desktop')
$racine = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_VERSION_FINALE (a ouvrir)') } | Select-Object -First 1).Path
$impr = Join-Path (Split-Path $racine) '02_Impression_3D'
$arch = Join-Path $racine '3_Anciennes versions\3k_08-11 etat avant rangement final\Impression'
New-Item -ItemType Directory -Force $arch | Out-Null
Get-ChildItem $impr -Filter '*.bgcode' | Move-Item -Destination $arch
$ch = Join-Path $impr 'Robot (chassis)'
if (Test-Path -LiteralPath $ch) {
    Get-ChildItem -LiteralPath $ch -File | Move-Item -Destination $impr -Force
    if (-not (Get-ChildItem -LiteralPath $ch -Force)) { Remove-Item -LiteralPath $ch }
}
$tmp = Join-Path $PSScriptRoot 'stl_export'
foreach ($f in Get-ChildItem $tmp -File) {
    $n = $f.Name
    if ($n -like 'roue*') { $n = $f.BaseName + ' x2.STL' }
    Copy-Item -LiteralPath $f.FullName -Destination (Join-Path $impr $n) -Force
}
Get-ChildItem $impr -Recurse | ForEach-Object { '{0,9} {1:dd/MM HH:mm} {2}' -f $_.Length, $_.LastWriteTime, $_.Name }
