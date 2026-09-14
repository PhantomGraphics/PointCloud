# Gaussian Point / PBVR multi-seed measurements

Build GSView before running. Generate the synthetic samples with
`../download_gs_samples.ps1` if necessary.

```powershell
.\run_gp_evaluation.ps1 -Configuration Debug -SeedCount 5 -Frames 128 -OutputDirectory C:\Dev\Crystal2024\scratch\gp-evaluation
```

Use a new output directory for every experiment. The runner executes GPS and
all three PBVR methods in separate processes for each seed. `-Model` accepts an
absolute path or a path relative to this script. `-Spp` accepts 1, 4, 9, or 16.

Artifacts:

- `manifest.json`: source revision and dirty flag, executable/model/shader SHA-256,
  and experiment parameters. The revision describes the checkout at execution;
  the executable and shader hashes identify the actual binaries.
- Per-run scenario JSON and stdout/stderr logs.
- `runs.json`: unaggregated profiles, including seed, GPU, raw vendor-specific
  driver version, accumulation count, particle counts, and pass timings.
- `summary.json`: sample mean, sample standard deviation and Student-t 95%
  confidence interval across seeds (conservative df=30 critical value for n>31).

`computeMs` is the last completed frame's GPU timing per process, not the mean
of all frames. `Frames` counts scenario polling steps; use recorded `accumFrames`
for the actual accumulation count. Timings require an otherwise idle GPU.
Two seeds are suitable only for a smoke test; use more for reported results.
Zero timings may indicate unsupported GPU timestamps.

`rendererBufferBytes` is the current requested size of this renderer's buffers,
including host-visible buffers. It excludes allocator overhead, temporary upload
allocations, other renderers, images and driver allocations. It is **not VRAM
peak**. `driverVersionRaw` must be decoded according to the GPU vendor.

This runner measures workload/performance. It does not compute image PSNR,
SSIM, LPIPS, temporal flicker or multi-view bias, and does not establish image
quality equivalence. Those remain separate evaluation work.

## Explicit random streams

`SetGpSeed:<uint32>` / `GetGpSeed` select a random stream in both GPS and PBVR.
Changing the value resets accumulation. Deterministic mode keeps that stream
fixed; frame mode varies it with the frame counter. Seed 0 preserves the previous
random sequence. Reproducing frame-mode results also requires the same command
sequence and frame schedule; setting the seed alone does not rewind the clock.

`scenarios/gaussian_point_seed.json` checks invalid inputs, the full uint32 range,
and restoration of the deterministic generated count after switching seeds.
