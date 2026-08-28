#Requires -Version 5.1
<#
.SYNOPSIS
  点群サンプルデータ 生成
  合成点群を PointCloud\samples\ に生成する（実データのダウンロードは行わない --
  実データ（Stanford Bunny/Armadillo/Happy Buddha/Dragon）での追加検証は
  ThirdPartyScenarios\PointCloudView\download_pointcloud_samples.ps1（非公開のルートリポジトリ側）を参照）。
  ソース定義: PointCloud\download_pointcloud_samples.json
#>
$ErrorActionPreference = 'Stop'
$ProgressPreference    = 'SilentlyContinue'

$outputDir = Join-Path $PSScriptRoot 'samples'
$config    = Get-Content -Path (Join-Path $PSScriptRoot 'download_pointcloud_samples.json') -Raw | ConvertFrom-Json
$samples   = @($config.samples)

# ---- 合成データ ----
function Get-Gauss([Random]$rng, [double]$sigma) {
    $u = [Math]::Max($rng.NextDouble(), 1e-15)
    $sigma * [Math]::Sqrt(-2 * [Math]::Log($u)) * [Math]::Cos(2 * [Math]::PI * $rng.NextDouble())
}

function Write-PlyAscii([string]$path, $points) {
    $sw = [IO.StreamWriter]::new($path, $false, [Text.Encoding]::ASCII)
    $sw.NewLine = "`n"
    $sw.WriteLine('ply'); $sw.WriteLine('format ascii 1.0')
    $sw.WriteLine("element vertex $($points.Count)")
    $sw.WriteLine('property float x'); $sw.WriteLine('property float y'); $sw.WriteLine('property float z')
    $sw.WriteLine('end_header')
    foreach ($p in $points) { $sw.WriteLine(('{0:F6} {1:F6} {2:F6}' -f $p[0], $p[1], $p[2])) }
    $sw.Close()
}

function New-SpherePoints([double]$radius, [int]$nPoints) {
    $rng = [Random]::new(); $pts = [Collections.Generic.List[double[]]]::new()
    for ($i = 0; $i -lt $nPoints; $i++) {
        $theta = [Math]::Acos(1 - 2 * $rng.NextDouble())
        $phi   = 2 * [Math]::PI * $rng.NextDouble()
        $pts.Add([double[]]@(($radius * [Math]::Sin($theta) * [Math]::Cos($phi)),
                   ($radius * [Math]::Sin($theta) * [Math]::Sin($phi)),
                   ($radius * [Math]::Cos($theta))))
    }
    ,$pts
}

function New-CylinderPoints([double]$radius, [double]$height, [int]$nPoints) {
    $rng = [Random]::new(); $pts = [Collections.Generic.List[double[]]]::new()
    for ($i = 0; $i -lt $nPoints; $i++) {
        $theta = 2 * [Math]::PI * $rng.NextDouble()
        $z     = $height * ($rng.NextDouble() - 0.5)
        $pts.Add([double[]]@(($radius * [Math]::Cos($theta)), ($radius * [Math]::Sin($theta)), $z))
    }
    ,$pts
}

function New-PlanePoints([double]$size, [int]$nPoints, [double]$noise) {
    $rng = [Random]::new(); $pts = [Collections.Generic.List[double[]]]::new()
    for ($i = 0; $i -lt $nPoints; $i++) {
        $pts.Add([double[]]@(($size * ($rng.NextDouble() - 0.5)),
                   ($size * ($rng.NextDouble() - 0.5)),
                   (Get-Gauss $rng $noise)))
    }
    ,$pts
}

function New-TorusPoints([double]$R, [double]$tubeR, [int]$nPoints) {
    $rng = [Random]::new(); $pts = [Collections.Generic.List[double[]]]::new()
    for ($i = 0; $i -lt $nPoints; $i++) {
        $u = 2 * [Math]::PI * $rng.NextDouble()
        $v = 2 * [Math]::PI * $rng.NextDouble()
        $pts.Add([double[]]@((($R + $tubeR * [Math]::Cos($v)) * [Math]::Cos($u)),
                   (($R + $tubeR * [Math]::Cos($v)) * [Math]::Sin($u)),
                   ($tubeR * [Math]::Sin($v))))
    }
    ,$pts
}

# 多octaveの角度依存半径変調を持つ球。閉じた(watertight)非凸形状で、解析プリミティブ
# (sphere/cylinder/plane/torus)より複雑な"有機的"表面が要る検証(実データダウンロード無しで
# ダウンサンプル/法線推定/大規模ロードパスを一通り検証できる)用の代替。
# 旧 Stanford Bunny / Happy Buddha の代替、docs/todo/PLAN_scenario_test_synthetic_assets.md Phase 2。
function New-OrganicPoints([double]$baseRadius, [int]$nPoints, [int]$octaves, [double]$ampScale, [int]$seed) {
    $rng = [Random]::new($seed)
    $pts = [Collections.Generic.List[double[]]]::new()
    for ($i = 0; $i -lt $nPoints; $i++) {
        $theta = [Math]::Acos(1 - 2 * $rng.NextDouble())
        $phi   = 2 * [Math]::PI * $rng.NextDouble()
        $r = $baseRadius
        for ($k = 1; $k -le $octaves; $k++) {
            $amp = $ampScale / $k
            $r += $baseRadius * $amp * [Math]::Sin($k * 2.0 * $theta + $k) * [Math]::Cos(($k + 1) * $phi - $k * 0.7)
        }
        if ($r -lt $baseRadius * 0.15) { $r = $baseRadius * 0.15 } # floor: guards against near-zero/negative radius when octave phases align
        $pts.Add([double[]]@(($r * [Math]::Sin($theta) * [Math]::Cos($phi)),
                   ($r * [Math]::Sin($theta) * [Math]::Sin($phi)),
                   ($r * [Math]::Cos($theta))))
    }
    ,$pts
}

# ---- メイン ----
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

Write-Host ('=' * 60)
Write-Host '点群サンプルデータ 生成'
Write-Host "出力先: $outputDir"
Write-Host ('=' * 60)

$results = [ordered]@{}
foreach ($s in $samples) {
    Write-Host "`n[$($s.name)]  $($s.desc)"

    $outPath = Join-Path $outputDir $s.output
    if (Test-Path $outPath) {
        Write-Host "  既存ファイル: $($s.output) -- スキップ"
        $results[$s.name] = 'CACHED'; continue
    }

    $p   = $s.params
    $pts = switch ($s.generator) {
        'sphere'   { New-SpherePoints   $p.radius $p.nPoints }
        'cylinder' { New-CylinderPoints $p.radius $p.height $p.nPoints }
        'plane'    { New-PlanePoints    $p.size   $p.nPoints $p.noise }
        'torus'    { New-TorusPoints    $p.majorRadius $p.tubeRadius $p.nPoints }
        'organic'  { New-OrganicPoints  $p.baseRadius $p.nPoints $p.octaves $p.ampScale $p.seed }
    }
    Write-Host "  $($pts.Count) 点 生成完了"
    Write-Host "  書き出し中: $($s.output) ..." -NoNewline
    Write-PlyAscii $outPath $pts
    Write-Host (' {0:N0} KB' -f ((Get-Item $outPath).Length / 1KB))
    $results[$s.name] = 'OK'
}

Write-Host "`n$('=' * 60)"
Write-Host '結果サマリ'
Write-Host ('=' * 60)
foreach ($kv in $results.GetEnumerator()) {
    $icon = @{ OK='OK  '; CACHED='CACHE' }[$kv.Value]
    Write-Host "  [$icon] $($kv.Key)"
}

Write-Host "`n完了。出力先: $outputDir"
