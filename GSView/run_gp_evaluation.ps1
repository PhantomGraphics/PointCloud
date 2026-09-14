# Reproducible multi-seed GPU measurements. Run from any working directory.
[CmdletBinding()]
param(
    [ValidateSet('Debug','Release')][string]$Configuration = 'Debug',
    [ValidateRange(2,100)][int]$SeedCount = 5,
    [ValidateRange(16,10000)][int]$Frames = 128,
    [ValidateSet(1,4,9,16)][int]$Spp = 4,
    [string]$Model = '../samples/gs/gs_sphere.ply',
    [string]$OutputDirectory = 'evaluation-results'
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
$manifest = [ordered]@{
    schemaVersion = 1; sourceCommit = $revision; sourceDirty = $dirty
    executableSha256 = (Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash
    model = $modelPath; modelSha256 = (Get-FileHash -LiteralPath $modelPath -Algorithm SHA256).Hash
    shaderSha256 = @(Get-ChildItem -LiteralPath (Join-Path (Split-Path $exe) 'shaders') -Filter 'gps_*.spv' | Sort-Object Name | ForEach-Object { @{ name=$_.Name; sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash } })
    seedCount = $SeedCount; frames = $Frames; spp = $Spp
    note = 'Per-run GPU timings are the last completed frame, not a frame-time distribution. No quality equivalence is inferred.'
}
$manifest | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $out 'manifest.json') -Encoding UTF8
$rows = @()
foreach ($method in @('GaussianPoint','proportional','extinction','view_conditioned')) {
    for ($seed = 0; $seed -lt $SeedCount; ++$seed) {
        $id = "${method}_seed${seed}"
        $steps = @(
            @{ command='SetRenderMode:GaussianPoint'; expect='OK' },
            @{ command="LoadPLY:$modelPath"; expect_prefix='OK:' },
            @{ command="SetGpSeed:$seed"; expect='OK' },
            @{ command="GetGpSeed"; expect="Val:$seed" },
            @{ command='SetGpSeedMode:frame'; expect='OK' },
            @{ command="SetGpSpp:$Spp"; expect='OK' },
            @{ command='SetGpPointBudget:0'; expect='OK' },
            @{ command='SetGpDensityScale:1'; expect='OK' }
        )
        if ($method -eq 'GaussianPoint') {
            $steps += @{ command='SetRenderMode:GaussianPoint'; expect='OK' }
        } else {
            $steps += @{ command="SetPbvr3dMethod:$method"; expect='OK' }
            $steps += @{ command='SetRenderMode:PBVR3DExperimental'; expect='OK' }
        }
        $steps += @{ command='GetStatus'; expect='OK'; repeat=$Frames }
        $steps += @{ command='GetGpGeneratedCount'; expect_prefix='Count:'; expect_not='Count:0' }
        $steps += @{ command='GetGpProfile'; expect_prefix='Profile:gpu=' }
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
        $row = [ordered]@{ experimentMethod=$method; seedIndex=$seed }
        foreach ($kv in ($line -split 'Profile:',2)[1].Split(';')) {
            $parts = $kv.Split('=',2)
            if ($parts.Count -eq 2) { $row[$parts[0]] = $parts[1] }
        }
        $rows += [pscustomobject]$row
        Write-Host "Measured $id"
    }
}
$rows | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $out 'runs.json') -Encoding UTF8
# Student-t 95% intervals across independent seeded runs (not across correlated frames).
$t95 = @(0,12.706,4.303,3.182,2.776,2.571,2.447,2.365,2.306,2.262,2.228,2.201,2.179,2.160,2.145,2.131,2.120,2.110,2.101,2.093,2.086,2.080,2.074,2.069,2.064,2.060,2.056,2.052,2.048,2.045,2.042)
$summary = foreach ($group in ($rows | Group-Object experimentMethod)) {
    foreach ($metric in @('computeMs','generated','candidatePoints','activeSamples','rendererBufferBytes')) {
        $values = @($group.Group | ForEach-Object { [double]::Parse($_.$metric,[Globalization.CultureInfo]::InvariantCulture) })
        $mean = ($values | Measure-Object -Average).Average
        $sum = 0.0
        foreach ($value in $values) { $sum += ($value - $mean) * ($value - $mean) }
        $sd = [Math]::Sqrt($sum / ($values.Count - 1))
        # df>30: df=30 is conservative.
        $half = $t95[[Math]::Min(30,$values.Count - 1)] * $sd / [Math]::Sqrt($values.Count)
        [pscustomobject]@{ method=$group.Name; metric=$metric; n=$values.Count; mean=$mean; stddev=$sd; ci95Low=$mean-$half; ci95High=$mean+$half }
    }
}
$summary | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $out 'summary.json') -Encoding UTF8
Write-Host "Results: $out"
