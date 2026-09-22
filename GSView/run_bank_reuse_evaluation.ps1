# Measures the PBVR3D particle-bank reuse speedup (PLAN_pbvr_gps_ensemble_lod.md
# Phase 4 completion condition: "re-use speedup and sample-correlation/cache error
# explained separately"). Unlike run_gp_evaluation.ps1 (static camera, GaussianPoint
# vs the 3 PBVR3D methods), this orbits the camera every frame -- the situation
# pbvrBankReuse is meant to speed up -- crossed with bank reuse on/off, for each
# PBVR3D method. Proportional/Extinction should show the reuse speedup with an
# unbiased image (their candidate count is view-independent, so drift invalidation
# from GaussianPointRenderer::recordCompute() never engages); ViewConditioned's
# small 0.1 rad/step orbit stays well within kViewConditionedBankDriftTolerance
# (GaussianPointRenderer.cpp).
#
# The orbit dance (settle 6 frames, then OrbitSteps discrete ~0.1 rad camera-only
# steps with one frame between each) mirrors scenarios/pbvr3d_bank_reuse.json.
# This headless harness advances exactly one rendered frame per dispatched
# scenario command, not only GetStatus -- and a soft reset's isResetFrame window
# only covers the 2 frames right after it (frames_ = kMaxFrames = 2). GetGpProfile
# must therefore be the very first command issued after the last camera move, or
# the profile read lands a frame past that window and bankReused reads back 0
# even though reuse is working correctly (discovered empirically while writing
# this script -- see its own trailing comment at the profile-read site).
# Run from any working directory.
[CmdletBinding()]
param(
    [ValidateSet('Debug','Release')][string]$Configuration = 'Debug',
    [ValidateRange(2,100)][int]$SeedCount = 5,
    [ValidateRange(4,100)][int]$OrbitSteps = 4,
    [ValidateSet(1,4,9,16)][int]$Spp = 4,
    [string]$Model = '../samples/gs/gs_sphere.ply',
    [string]$OutputDirectory = 'evaluation-results-bank-reuse'
)
$ErrorActionPreference = 'Stop'
# Windows PowerShell may inherit both Path and PATH from the host.
$evaluationPath = $env:Path
Remove-Item Env:PATH -ErrorAction SilentlyContinue
$env:Path = $evaluationPath
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$exe = Join-Path $root "build/windows-$($Configuration.ToLower())/PointCloud/GSView.exe"
if (!(Test-Path -LiteralPath $exe)) { throw "Build GSView first: $exe" }
$modelPath = if ([IO.Path]::IsPathRooted($Model)) { $Model } else { Join-Path $PSScriptRoot $Model }
$modelPath = (Resolve-Path -LiteralPath $modelPath).Path
$out = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutputDirectory)
# Never mix a new experiment with stale files.
if (Test-Path -LiteralPath $out) { throw "Use a new output directory: $out" }
[IO.Directory]::CreateDirectory($out) | Out-Null
$revision = (& git -C $PSScriptRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE) { throw 'Cannot identify source revision' }
$dirty = [bool](& git -C $PSScriptRoot status --porcelain --untracked-files=no)
# 0.1 rad/step, matching scenarios/pbvr3d_bank_reuse.json's proven orbit -- large
# enough that each SetGpCamera reliably lands as its own distinct camera-changed
# frame, small enough to stay inside setEvaluationCamera's (0.01, 3.13) range for
# up to 100 steps starting at theta=1.0.
$thetaStep = 0.1
$manifest = [ordered]@{
    schemaVersion = 1; sourceCommit = $revision; sourceDirty = $dirty
    executableSha256 = (Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash
    model = $modelPath; modelSha256 = (Get-FileHash -LiteralPath $modelPath -Algorithm SHA256).Hash
    shaderSha256 = @(Get-ChildItem -LiteralPath (Join-Path (Split-Path $exe) 'shaders') -Filter 'gps_*.spv' | Sort-Object Name | ForEach-Object { @{ name=$_.Name; sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash } })
    seedCount = $SeedCount; orbitSteps = $OrbitSteps; spp = $Spp; thetaStep = $thetaStep
    note = 'The reported profile is the frame right after the last orbit step (steady-state reuse or steady-state resample), not a frame-time distribution. No image-quality equivalence is inferred.'
}
$manifest | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $out 'manifest.json') -Encoding UTF8
$rows = @()
foreach ($method in @('proportional','extinction','view_conditioned')) {
    foreach ($bankReuse in @(0,1)) {
        for ($seed = 0; $seed -lt $SeedCount; ++$seed) {
            $id = "${method}_reuse${bankReuse}_seed${seed}"
            $steps = @(
                @{ command="LoadPLY:$modelPath"; expect_prefix='OK:' },
                @{ command="SetGpSeed:$seed"; expect='OK' },
                @{ command="GetGpSeed"; expect="Val:$seed" },
                @{ command='SetGpSeedMode:frame'; expect='OK' },
                @{ command="SetGpSpp:$Spp"; expect='OK' },
                @{ command='SetGpPointBudget:0'; expect='OK' },
                @{ command='SetGpDensityScale:1'; expect='OK' },
                @{ command='SetGpCompact:1'; expect='OK' },
                @{ command="SetPbvr3dMethod:$method"; expect='OK' },
                @{ command='SetRenderMode:PBVR3DExperimental'; expect='OK' },
                @{ command="SetPbvrBankReuse:$bankReuse"; expect='OK' },
                @{ command='SetGpCamera:1.0,0.7,3'; expect='OK' },
                @{ command='GetStatus'; expect='OK'; repeat=6 }
            )
            # Orbit dance matching scenarios/pbvr3d_bank_reuse.json: OrbitSteps-1
            # camera-only steps each followed by one frame, then a final step with
            # no trailing frame before reading the profile (so the profile reflects
            # the frame that decided reuse-vs-resample for that final camera move).
            for ($i = 1; $i -le $OrbitSteps; ++$i) {
                $theta = [Math]::Round(1.0 + $thetaStep * $i, 6)
                $steps += @{ command="SetGpCamera:$theta,0.7,3"; expect='OK' }
                if ($i -lt $OrbitSteps) { $steps += @{ command='GetStatus'; expect='OK' } }
            }
            # GetGpProfile must be the FIRST command issued after the final camera
            # move: this headless harness advances one rendered frame per
            # dispatched command (not only GetStatus), and a soft reset's
            # isResetFrame window only covers the 2 frames right after it
            # (frames_ = kMaxFrames = 2, one per double-buffered slot). Reading
            # GetGpGeneratedCount first before GetGpProfile pushed the profile
            # read a frame too late and made bankReused read back 0 even when
            # reuse was working (discovered empirically -- see the commit that
            # added this script).
            $steps += @{ command='GetGpProfile'; expect_prefix='Profile:gpu=' }
            $steps += @{ command='GetGpGeneratedCount'; expect_prefix='Count:'; expect_not='Count:0' }
            $scenario = Join-Path $out "$id.json"
            [IO.File]::WriteAllText($scenario, (@{ name=$id; steps=$steps } | ConvertTo-Json -Depth 8), [Text.UTF8Encoding]::new($false))
            $log = Join-Path $out "$id.stdout.log"
            $err = Join-Path $out "$id.stderr.log"
            $process = Start-Process -FilePath $exe -ArgumentList @('--run-scenario',('"' + $scenario + '"')) -WorkingDirectory $PSScriptRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput $log -RedirectStandardError $err
            $null = $process.Handle
            $deadline = [DateTime]::UtcNow.AddMinutes(10)
            while (!$process.WaitForExit(1000)) {
                if ([DateTime]::UtcNow -gt $deadline) { $process.Kill(); throw "Timed out: $id" }
            }
            $process.WaitForExit()
            if ($process.ExitCode -ne 0) { throw "Failed: $id; see $err" }
            if (Select-String -LiteralPath $err -Pattern 'Validation Error|VUID-' -Quiet) { throw "Vulkan validation failure: $err" }
            $line = @(Select-String -LiteralPath $log -Pattern '< Profile:')[0].Line
            if (!$line) { throw "Missing profile: $log" }
            $row = [ordered]@{ experimentMethod=$method; bankReuseRequested=$bankReuse; seedIndex=$seed }
            foreach ($kv in ($line -split 'Profile:',2)[1].Split(';')) {
                $parts = $kv.Split('=',2)
                if ($parts.Count -eq 2) { $row[$parts[0]] = $parts[1] }
            }
            $rows += [pscustomobject]$row
            Write-Host "Measured $id (bankReused=$($row.bankReused))"
        }
    }
}
$rows | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $out 'runs.json') -Encoding UTF8
# Student-t 95% intervals across independent seeded runs (not across correlated frames).
$t95 = @(0,12.706,4.303,3.182,2.776,2.571,2.447,2.365,2.306,2.262,2.228,2.201,2.179,2.160,2.145,2.131,2.120,2.110,2.101,2.093,2.086,2.080,2.074,2.069,2.064,2.060,2.056,2.052,2.048,2.045,2.042)
$summary = foreach ($group in ($rows | Group-Object experimentMethod, bankReuseRequested)) {
    foreach ($metric in @('computeMs','depthMs','colorMs','generated','rendererBufferBytes')) {
        $values = @($group.Group | ForEach-Object { [double]::Parse($_.$metric,[Globalization.CultureInfo]::InvariantCulture) })
        $mean = ($values | Measure-Object -Average).Average
        $sum = 0.0
        foreach ($value in $values) { $sum += ($value - $mean) * ($value - $mean) }
        $sd = [Math]::Sqrt($sum / ($values.Count - 1))
        # df>30: df=30 is conservative.
        $half = $t95[[Math]::Min(30,$values.Count - 1)] * $sd / [Math]::Sqrt($values.Count)
        [pscustomobject]@{ group=$group.Name; metric=$metric; n=$values.Count; mean=$mean; stddev=$sd; ci95Low=$mean-$half; ci95High=$mean+$half }
    }
    $reusedCount = @($group.Group | Where-Object { $_.bankReused -eq '1' }).Count
    [pscustomobject]@{ group=$group.Name; metric='bankReusedFraction'; n=$group.Group.Count; mean=($reusedCount / $group.Group.Count); stddev=$null; ci95Low=$null; ci95High=$null }
}
$summary | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $out 'summary.json') -Encoding UTF8
Write-Host "Results: $out"
