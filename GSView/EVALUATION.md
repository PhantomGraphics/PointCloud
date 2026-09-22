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

## Pbvr3d particle-bank reuse speedup (PLAN_pbvr_gps_ensemble_lod.md Phase 4)

```powershell
.\run_bank_reuse_evaluation.ps1 -Configuration Release -SeedCount 8 -OutputDirectory C:\Dev\Crystal2024\scratch\bank-reuse
```

**Run this in Release, not Debug** -- Debug's unoptimized compute shaders and
validation overhead dominate the small per-dispatch cost this measures, hiding
whatever reuse-vs-resample difference exists.

Crosses all 3 PBVR3D methods with `pbvrBankReuse` on/off, orbiting the camera
every frame after settling (the situation reuse targets) instead of holding a
static camera like `run_gp_evaluation.ps1`. `summary.json`'s `bankReusedFraction`
confirms reuse actually engaged for the `bankReuse=1` runs (it must read `1.0`;
`0.0` for the `bankReuse=0` baseline runs is expected).

Measured on this machine's GPU (Intel Iris Xe, `gs_sphere.ply`, 2000 splats,
Spp=4, 8 seeds, Release): `computeMs` for `bankReuse=1` was **not** lower than
`bankReuse=0` for any of the 3 methods (proportional 8.75 vs 8.40 ms, extinction
10.85 vs 9.84 ms, view_conditioned 10.93 vs 10.89 ms) -- reuse showed no
measurable speedup here, and extinction/view_conditioned were mildly slower.
Reprojection (`gps_pbvr3d.comp` pass 3/4) skips `gps_compact.comp`'s
prepare/scan/compact stages but still re-runs the full per-particle depth+colour
splat over every bank entry, and on this dense a scene (700K-1.3M particles)
that splat, not prepare/scan/compact, is apparently the dominant cost -- so
avoiding regeneration barely moves `computeMs`. This does not mean the feature
is pointless (its purpose is also motion consistency: the ensemble LOD
controller forces R=1 while moving regardless, so at minimum reuse is a
zero-cost-when-not-helping optimization), but the speedup itself is unconfirmed
on this hardware/scene and should not be assumed without measuring on the
target configuration. A different-shaped bottleneck (very high per-Gaussian
`maxPointsPerSplat`, i.e. an expensive prepare/scan/compact relative to a sparse
splat) is a more promising case to re-measure.

**Repeated on the real 'train' scene (Tanks & Temples, 559,263 Gaussians, SH
degree 3 -- the dataset the GPS/3DGS literature evaluates on,
`docs/paper/gps_pbvr_2026/download_train.py`) with the same conclusion**: see
`docs/paper/gps_pbvr_2026/results_bank_reuse_train/README.md` and its own
`bank_reuse_train_evaluation.py`. `computeMs` was again statistically flat
between `bankReuse` on/off for all 3 methods (proportional 24.71 vs 24.61 ms,
extinction 34.44 vs 34.78 ms, view_conditioned 31.37 vs 31.31 ms, 8 seeds each),
ruling out "the synthetic scene was just too small" as the reason no speedup
showed up above. That script also documents a real-dataset-specific pitfall:
`LoadPLY` before `SetRenderMode:PBVR3DExperimental` renders the first several
frames in the default SortBased mode, whose per-frame resort
(`VkGSPointRenderer::sortByView`) is an O(n)-pass odd-even transposition sort --
fine at the <=15000-point synthetic samples every other scenario in this repo
uses, but O(n^2) and multi-minutes-per-frame at this scene's ~560K points.
Switch render mode before loading real-scale data.

**Correctness note (found and fixed while writing this measurement):**
`gps_compact.comp`'s generate pass only writes `bank[index]` for candidates
ViewConditioned's keep-probability *accepted*; a rejected candidate's slot was
left untouched, and reprojection iterated every slot up to the pre-thinning
total (`work[3]`), silently replaying whatever stale bank contents (an earlier
ensemble's, or an earlier epoch's) happened to be sitting in the rejected slots
as phantom particles. `GetGpProfile`'s `generated` count during ViewConditioned
reuse matched Extinction's *un-thinned* candidate total almost exactly instead
of ViewConditioned's own thinned baseline, which is what surfaced it. Fixed by
writing an explicit `0xFFFFFFFFu` id sentinel to rejected slots
(`gps_compact.comp`) and skipping it in `reprojectOne()` (`gps_pbvr3d.comp`).
Proportional/Extinction were never affected (they have no per-particle
rejection). This means every prior `pbvrBankReuse=1` + ViewConditioned frame
before this fix rendered roughly 2x too many particles -- a real image
correctness bug, not just a missed optimization.

**Build-system note (found while chasing the above):** a `TARGET POST_BUILD`
custom command only re-runs when the target itself relinks under the Ninja
generator. A GLSL-only edit recompiles `${target}_shaders` but touches no
`.obj`/`.lib`, so the exe was considered up to date and the copy to
`<exe dir>/shaders/*.spv` -- what the exe actually loads at runtime -- was
silently skipped; both the Debug and Release builds used while diagnosing the
bug above were still running the pre-fix shader despite `cmake --build`
reporting a successful recompile. Fixed in `cmake/PhantomVulkanApp.cmake`
(`phantom_add_runtime_shaders()`) by routing the copy through its own
OUTPUT/DEPENDS custom command keyed on the compiled `.spv` files, independent
of the exe's link state. This affects every Vulkan app built with that macro
(PhysicsView, PointCloudView, GSView, ...), not just this measurement -- a
GLSL-only change anywhere in this codebase could previously ship stale shaders
without a full/clean rebuild.
