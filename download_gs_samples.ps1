#Requires -Version 5.1
<#
.SYNOPSIS
  Gaussian Splatting サンプル PLY / .splat 生成
  合成 GS シーンを binary_little_endian PLY と .splat の両形式で生成する（実データのダウンロードは行わない）。
  ソース定義: PointCloud\download_gs_samples.json
  出力先: PointCloud\samples\gs\
#>
[CmdletBinding()] param()
$ErrorActionPreference = 'Stop'
$ProgressPreference    = 'SilentlyContinue'

$outputDir = Join-Path $PSScriptRoot 'samples\gs'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$config = Get-Content -Path (Join-Path $PSScriptRoot 'download_gs_samples.json') -Raw | ConvertFrom-Json

$props = 'x','y','z','nx','ny','nz','f_dc_0','f_dc_1','f_dc_2','opacity',
         'scale_0','scale_1','scale_2','rot_0','rot_1','rot_2','rot_3'

# ---- GS バイナリ PLY ライター ----
function Write-GSPly([string]$dest, $points) {
    $n = $points.Count
    $header = "ply`nformat binary_little_endian 1.0`nelement vertex $n`n"
    foreach ($p in $props) { $header += "property float $p`n" }
    $header += "end_header`n"
    $stream = [IO.FileStream]::new($dest, [IO.FileMode]::Create)
    $hBytes = [Text.Encoding]::ASCII.GetBytes($header)
    $stream.Write($hBytes, 0, $hBytes.Length)
    $bw = [IO.BinaryWriter]::new($stream)
    foreach ($pt in $points) {
        foreach ($p in $props) { $bw.Write([float]$pt[$p]) }
    }
    $bw.Close()
    Write-Host ("  -> saved: $dest  ({0} splats, {1:F0} kB)" -f $n, ((Get-Item $dest).Length / 1KB))
}

# ---- .splat（32バイト/スプラットのフラットバイナリ）ライター ----
# GSPointCloud::readFromFile() の .splat デコード（GSPointCloud.cpp）の逆演算でエンコードする。
function Write-GSSplat([string]$dest, $points) {
    $n  = $points.Count
    $C0 = 0.28209479177387814
    $stream = [IO.FileStream]::new($dest, [IO.FileMode]::Create)
    $bw = [IO.BinaryWriter]::new($stream)
    foreach ($pt in $points) {
        $bw.Write([float]$pt.x); $bw.Write([float]$pt.y); $bw.Write([float]$pt.z)
        $bw.Write([float][Math]::Exp($pt.scale_0))
        $bw.Write([float][Math]::Exp($pt.scale_1))
        $bw.Write([float][Math]::Exp($pt.scale_2))

        $r = [Math]::Round([Math]::Max(0.0, [Math]::Min(1.0, $pt.f_dc_0 * $C0 + 0.5)) * 255)
        $g = [Math]::Round([Math]::Max(0.0, [Math]::Min(1.0, $pt.f_dc_1 * $C0 + 0.5)) * 255)
        $b = [Math]::Round([Math]::Max(0.0, [Math]::Min(1.0, $pt.f_dc_2 * $C0 + 0.5)) * 255)
        $sigOpacity = 1.0 / (1.0 + [Math]::Exp(-$pt.opacity))
        $a = [Math]::Round($sigOpacity * 255)
        $bw.Write([byte]$r); $bw.Write([byte]$g); $bw.Write([byte]$b); $bw.Write([byte]$a)

        $rw = [Math]::Round([Math]::Max(0.0, [Math]::Min(255.0, $pt.rot_0 * 128 + 128)))
        $rx = [Math]::Round([Math]::Max(0.0, [Math]::Min(255.0, $pt.rot_1 * 128 + 128)))
        $ry = [Math]::Round([Math]::Max(0.0, [Math]::Min(255.0, $pt.rot_2 * 128 + 128)))
        $rz = [Math]::Round([Math]::Max(0.0, [Math]::Min(255.0, $pt.rot_3 * 128 + 128)))
        $bw.Write([byte]$rw); $bw.Write([byte]$rx); $bw.Write([byte]$ry); $bw.Write([byte]$rz)
    }
    $bw.Close()
    Write-Host ("  -> saved: $dest  ({0} splats, {1:F0} kB)" -f $n, ((Get-Item $dest).Length / 1KB))
}

function New-Splat([double]$x,[double]$y,[double]$z,[double]$nx,[double]$ny,[double]$nz,
                   [double]$r,[double]$g,[double]$b,
                   [double]$sx=0.015,[double]$sy=0.015,[double]$sz=0.004) {
    [ordered]@{
        x=$x; y=$y; z=$z; nx=$nx; ny=$ny; nz=$nz
        f_dc_0=$r; f_dc_1=$g; f_dc_2=$b; opacity=[double]1
        scale_0=[Math]::Log($sx); scale_1=[Math]::Log($sy); scale_2=[Math]::Log($sz)
        rot_0=[double]1; rot_1=[double]0; rot_2=[double]0; rot_3=[double]0
    }
}

function Get-Gauss([Random]$rng, [double]$sigma) {
    $u = [Math]::Max($rng.NextDouble(), 1e-15)
    $sigma * [Math]::Sqrt(-2 * [Math]::Log($u)) * [Math]::Cos(2 * [Math]::PI * $rng.NextDouble())
}

function New-SpherePoints([int]$n = 2000, [double]$radius = 1.0) {
    $rng = [Random]::new(42)
    $pts = [Collections.Generic.List[object]]::new()
    while ($pts.Count -lt $n) {
        $u = $rng.NextDouble() * 2 - 1; $v = $rng.NextDouble() * 2 - 1
        $s = $u * $u + $v * $v
        if ($s -ge 1.0) { continue }
        $t = 2 * [Math]::Sqrt(1 - $s); $nx = $u * $t; $ny = $v * $t; $nz = 1 - 2 * $s
        $pts.Add((New-Splat ($nx * $radius) ($ny * $radius) ($nz * $radius) $nx $ny $nz `
            (($nx + 1) * 0.14) (($ny + 1) * 0.14) (($nz + 1) * 0.14)))
    }
    ,$pts
}

function New-CubePoints([int]$nPerFace = 500, [double]$half = 1.0) {
    $rng  = [Random]::new(7)
    $pts  = [Collections.Generic.List[object]]::new()
    $faces = @(@(1,0,0),@(-1,0,0),@(0,1,0),@(0,-1,0),@(0,0,1),@(0,0,-1))
    foreach ($face in $faces) {
        $fnx = $face[0]; $fny = $face[1]; $fnz = $face[2]
        for ($i = 0; $i -lt $nPerFace; $i++) {
            $u = $rng.NextDouble() * 2 * $half - $half
            $v = $rng.NextDouble() * 2 * $half - $half
            if ($fnx -ne 0)     { $x = [double]$fnx * $half; $y = $u; $z = $v }
            elseif ($fny -ne 0) { $x = $u; $y = [double]$fny * $half; $z = $v }
            else                { $x = $u; $y = $v; $z = [double]$fnz * $half }
            $pts.Add((New-Splat $x $y $z ([double]$fnx) ([double]$fny) ([double]$fnz) `
                (($fnx + 1) * 0.14) (($fny + 1) * 0.14) (($fnz + 1) * 0.14) 0.015 0.015 0.003))
        }
    }
    ,$pts
}

function New-GalaxyPoints([int]$n = 3000, [int]$arms = 3) {
    $rng = [Random]::new(99)
    $pts = [Collections.Generic.List[object]]::new()
    for ($i = 0; $i -lt $n; $i++) {
        $armPhase = ($i % $arms) * (2 * [Math]::PI / $arms)
        $t     = $rng.NextDouble()
        $r     = $t * 2
        $theta = $t * 4 * [Math]::PI + $armPhase
        $x     = $r * [Math]::Cos($theta) + (Get-Gauss $rng 0.05)
        $z     = $r * [Math]::Sin($theta) + (Get-Gauss $rng 0.05)
        $y     = (Get-Gauss $rng 0.04) * (1 - $t * 0.8)
        $pts.Add([ordered]@{
            x=$x; y=$y; z=$z; nx=[double]0; ny=[double]1; nz=[double]0
            f_dc_0=$t*0.28; f_dc_1=(1-$t)*0.15; f_dc_2=(1-$t)*0.28
            opacity=$rng.NextDouble()*0.7+0.8
            scale_0=[Math]::Log(0.01+$rng.NextDouble()*0.03)
            scale_1=[Math]::Log(0.01+$rng.NextDouble()*0.03)
            scale_2=[Math]::Log(0.002+$rng.NextDouble()*0.008)
            rot_0=[double]1; rot_1=[double]0; rot_2=[double]0; rot_3=[double]0
        })
    }
    ,$pts
}

function New-TorusPoints([int]$n = 3000, [double]$R = 1.5, [double]$tubeR = 0.4) {
    $rng = [Random]::new(55)
    $pts = [Collections.Generic.List[object]]::new()
    for ($i = 0; $i -lt $n; $i++) {
        $phi   = $rng.NextDouble() * 2 * [Math]::PI
        $theta = $rng.NextDouble() * 2 * [Math]::PI
        $nx = [Math]::Cos($phi) * [Math]::Cos($theta)
        $ny = [Math]::Sin($phi) * [Math]::Cos($theta)
        $nz = [Math]::Sin($theta)
        $x  = $R * [Math]::Cos($phi) + $tubeR * $nx
        $y  = $R * [Math]::Sin($phi) + $tubeR * $ny
        $z  = $tubeR * $nz
        $pts.Add((New-Splat $x $y $z $nx $ny $nz `
            (([Math]::Cos($phi) + 1) * 0.14) (([Math]::Sin($phi) + 1) * 0.14) (([Math]::Cos($theta) + 1) * 0.14) `
            0.02 0.02 0.005))
    }
    ,$pts
}

# 対数螺旋の巻殻（テーパー付きチューブ断面）。非対称・大規模な形状のロードパス検証用。
function New-ShellPoints([int]$n = 15000, [double]$turns = 4.5, [double]$baseRadius = 0.08,
                         [double]$growth = 0.18, [double]$pitch = 0.05, [double]$tubeRadius = 0.05) {
    $rng = [Random]::new(123)
    $pts = [Collections.Generic.List[object]]::new()
    $maxTheta = $turns * 2 * [Math]::PI
    for ($i = 0; $i -lt $n; $i++) {
        $t = $rng.NextDouble() * $maxTheta
        $r = $baseRadius * [Math]::Exp($growth * $t)
        $cx = $r * [Math]::Cos($t)
        $cy = $r * [Math]::Sin($t)
        $cz = $pitch * $t

        $tubeR = $tubeRadius * (1.0 - 0.6 * ($t / $maxTheta))
        $phi   = $rng.NextDouble() * 2 * [Math]::PI
        $rad   = [Math]::Sqrt($rng.NextDouble()) * $tubeR
        $ux = [Math]::Cos($t); $uy = [Math]::Sin($t)
        $ox = $rad * [Math]::Cos($phi) * $ux
        $oy = $rad * [Math]::Cos($phi) * $uy
        $oz = $rad * [Math]::Sin($phi)

        $x = $cx + $ox; $y = $cy + $oy; $z = $cz + $oz
        $nx = [Math]::Cos($phi) * $ux; $ny = [Math]::Cos($phi) * $uy; $nz = [Math]::Sin($phi)
        $shade = $t / $maxTheta
        $pts.Add((New-Splat $x $y $z $nx $ny $nz `
            (0.55 + 0.25 * $shade) (0.35 + 0.15 * (1 - $shade)) (0.15 + 0.35 * $shade) `
            ($tubeR * 0.3) ($tubeR * 0.3) ($tubeR * 0.12)))
    }
    ,$pts
}

# ---- メイン ----
Write-Host ('=' * 55)
Write-Host '  Gaussian Splatting Sample Generator'
Write-Host ('=' * 55)
Write-Host "  Output dir : $outputDir`n"

Write-Host 'Generating synthetic GS PLY / .splat files'
foreach ($entry in @($config.synthetic)) {
    $dest      = Join-Path $outputDir $entry.file
    $splatDest = Join-Path $outputDir ([IO.Path]::ChangeExtension($entry.file, '.splat'))
    $needPly   = -not (Test-Path $dest)
    $needSplat = -not (Test-Path $splatDest)

    if (-not $needPly -and -not $needSplat) {
        Write-Host ("  [skip] {0}  ({1} kB, already exists)" -f $entry.file, ([int]((Get-Item $dest).Length / 1KB)))
        continue
    }

    Write-Host "  Generating $($entry.label) ..."
    $p   = $entry.params
    $pts = switch ($entry.generator) {
        'sphere' { New-SpherePoints $p.n $p.radius }
        'cube'   { New-CubePoints   $p.nPerFace $p.half }
        'galaxy' { New-GalaxyPoints $p.n $p.arms }
        'torus'  { New-TorusPoints  $p.n $p.majorRadius $p.tubeRadius }
        'shell'  { New-ShellPoints  $p.n $p.turns $p.baseRadius $p.growth $p.pitch $p.tubeRadius }
    }
    if ($needPly)   { Write-GSPly   $dest      $pts }
    if ($needSplat) { Write-GSSplat $splatDest $pts }
}

Write-Host ''
Write-Host 'Done.  Files in output directory:'
Get-ChildItem -Path $outputDir -File | Where-Object { $_.Extension -in '.ply', '.splat' } | Sort-Object Name | ForEach-Object {
    Write-Host ("  {0,-24}  {1,6} kB" -f $_.Name, ([int]($_.Length / 1KB)))
}
