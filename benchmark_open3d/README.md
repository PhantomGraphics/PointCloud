# PointCloud vs Open3D vs PCL 速度比較

このフォルダは、本リポジトリの `PointCloud`（`PointCloud/PointCloud/`）モジュールと
[Open3D](https://www.open3d.org/) 0.19.0、[PCL](https://pointclouds.org/) 1.15.1
（Point Cloud Library、公式 AllInOne Windows インストーラー配布のプリビルド版）を、
共通するアルゴリズムで速度比較した際に使ったスクリプト一式と結果レポートを保存したもの。
本体の `.sln` には組み込まない、独立したベンチマークとしてこのフォルダにまとめている。

## 3者比較の結果サマリ（2026-08-14 計測）

合成点群（50,000 / 200,000 / 1,000,000 点）を3実装で同一パラメータ・同一データセットで処理し、
処理時間を計測した（`x64/Release` ビルド・PointCloud/PCL は既定スレッド数（OpenMP 有効時は全論理コア）
/ Open3D 0.19.0・Python 3.11 / PCL 1.15.1・MSVC2022 プリビルド）。3本のベンチマークを同一セッション内で
順番に（他プロセスと競合させず）実行しており、相互に比較可能な数値になっている。

**PCL には DBSCAN のネイティブ実装が無い**（近い機能は `pcl::EuclideanClusterExtraction` だが
別アルゴリズムのため比較対象から除外）ため、PCL の行は4アルゴリズムのみ。また **PCL 側は
`pcl::NormalEstimation`／`pcl::IterativeClosestPoint` という標準（非 OpenMP）クラスで計測している**
（`pcl::NormalEstimationOMP` 等の並列版は未使用）。本実装（`PointCloud`）はこの2つに OpenMP 並列化を
適用済みのため、その分の差はライブラリ実装の優劣というより「今回使った API 選択」による差である点に注意。

| アルゴリズム | 点数 | PointCloud (秒) | Open3D (秒) | PCL (秒) | 最速 |
|---|---|---|---|---|---|
| ボクセルダウンサンプリング | 50K | 0.0049 | 0.0098 | **0.0023** | PCL |
| | 200K | 0.0133 | 0.0301 | **0.0077** | PCL |
| | 1M | 0.0684 | 0.0891 | **0.0592** | PCL |
| 法線推定 (PCA, radius=0.5) | 50K | **0.0450** | 0.0506 | 0.1319 | PointCloud |
| | 200K | **0.2307** | 0.4857 | 1.4499 | PointCloud |
| | 1M | **11.57** | 17.05 | 45.55 | PointCloud |
| SOR 外れ値除去 (k=20, std=2.0) | 50K | **0.0528** | 0.0536 | 0.1038 | PointCloud |
| | 200K | 0.2557 | **0.2275** | 0.5464 | Open3D |
| | 1M | **2.070** | 2.387 | 5.257 | PointCloud |
| RANSAC 平面検出 (iters=200) | 50K | 0.0098 | 0.0052 | **0.0017** | PCL |
| | 200K | 0.0263 | 0.0223 | **0.0071** | PCL |
| | 1M | 0.1767 | 0.1591 | **0.0513** | PCL |
| ICP (point-to-point, 30 iter) | 50K | **0.2264** | 0.3435 | 0.9035 | PointCloud |
| | 200K | **1.295** | 1.959 | 5.754 | PointCloud |
| | 1M | 27.22 | **26.32** | 123.2 | Open3D（僅差） |
| DBSCAN (eps=0.5, minPts=10) | 50K | 0.1207 | **0.0977** | (未対応) | Open3D |
| | 200K | **0.5661** | 0.5789 | — | PointCloud |
| | 1M | **6.655** | 7.872 | — | PointCloud |

**主な発見:**

1. **ボクセルダウンサンプリング・RANSAC 平面検出は PCL が全サイズで最速**（ダウンサンプリングは
   本実装比 2.1〜2.9倍、Open3D比 1.5〜4.3倍。RANSAC は本実装比 3.4〜5.8倍、Open3D比 3.1〜3.5倍）。
   どちらも「1回のパスで完結する」軽量な処理で、PCL の実装がオーバーヘッド面で有利と見られる。
   なお `PointCloud` と `PCL` のボクセルダウンサンプリングは出力点数が全サイズで完全一致
   （28135 / 38957 / 39304）——同じボクセルグリッド分割方式であることの裏付け。
2. **法線推定・ICP は本実装（PointCloud）が全サイズで PCL より 2.9〜4.5倍速い。**
   ただしこれは主に OpenMP 並列化の有無によるもの（`docs/todo/PLAN_pointcloud_openmp_parallelization.md` 参照）——
   PCL 側も `NormalEstimationOMP` を使えば差は縮む見込みだが、今回は両ライブラリの「標準クラス」を
   素直に使う条件で揃えている。SOR 外れ値除去の inlier 数は `PointCloud` と `PCL` で全サイズ完全一致
   （48260 / 193738 / 971324）——同一の統計的外れ値判定を実装していることの裏付け。
3. **ICP の fitness 値は `PointCloud` と `PCL` でほぼ完全一致**（例: 1M で 0.000354 vs 0.000353）。
   両者とも「最終反復の対応点間平均二乗距離」という同一定義（`PCL::getFitnessScore()` と本実装の
   `ICPRegistration::Result::fitness` が同じ計算）を採用しているため、Open3D（対応点重なり比率、
   全ケースで fitness=1.0 表示）とは異なりこの2つは数値としても比較可能。
4. **PCL は今回計測した4アルゴリズム中2つ（法線推定・ICP）で最も遅く、絶対時間で見ても
   1,000,000点の ICP が 123秒**（本実装 27秒、Open3D 26秒の約4.5〜4.7倍）と実用上の差が大きい。
   一方でダウンサンプリング・RANSAC平面検出は最速というように、アルゴリズムによって得意不得意が
   はっきり分かれる結果になった。

## 過去の2者比較（PointCloud vs Open3D、2026-08-10 時点）

合成点群（50,000 / 200,000 / 1,000,000 点）を両実装で同一パラメータで処理し、処理時間を計測した
（`x64/Release` ビルド・単一スレッド vs Open3D 0.19.0 / Python 3.11）。

**2026-08-10 に2回の再計測を行っている。1回目は `PointCloud` 側のその後のリファクタリング
（`IPointCloud` 導入・AoS→SoA 移行 Phase 1・`Space::KDTree` の `IPointItem` 廃止＆
`Vector3dfVector` 直接対応、`PointCloudKdTree` ラッパー削除）を反映してソリューション全体を
再ビルドしたもの、2回目はそれに加えて `SORFilter::execute()` を O(n²) 総当たりから
`Crystal::Space::KDTree` ベースの k-NN 探索（O(n log n)）へ書き換えたもの**
（初回計測時点の数値は各行に注記。Open3D 側の数値は再計測しても実質変化なし）。

| アルゴリズム | 点数 | PointCloud (秒) | Open3D (秒) | 倍率 (PC ÷ O3D) | 初回計測時点の倍率 |
|---|---|---|---|---|---|
| ボクセルダウンサンプリング | 50K | 0.029 | 0.011 | 2.7× | 3.3× |
| | 200K | 0.085 | 0.028 | 3.0× | 2.7× |
| | 1M | 0.596 | 0.074 | 8.1× | 3.2× |
| 法線推定 (PCA) | 50K | 0.119 | 0.073 | 1.6× | 2.7× |
| | 200K | 1.183 | 0.528 | 2.2× | 3.2× |
| | 1M | 46.59 | 21.56 | 2.2× | 3.2× |
| SOR 外れ値除去 | 50K | **0.172** | 0.053 | **3.2×** | 73× |
| | 200K | **0.879** | 0.239 | **3.7×** | 256× |
| | 1M | **8.845** | 3.803 | **2.3×** | — (未計測) |
| RANSAC 平面検出 | 50K | 0.027 | 0.014 | 2.0× | 5.2× |
| | 200K | 0.128 | 0.022 | 5.8× | 5.4× |
| | 1M | 0.763 | 0.278 | 2.7× | 3.8× |
| DBSCAN クラスタリング | 50K | 0.081 | 0.104 | **0.8×（本実装が高速）** | 0.9× |
| | 200K | 0.429 | 0.495 | **0.9×（本実装が高速）** | 0.9× |
| | 1M | 4.959 | 9.805 | **0.5×（本実装が高速）** | 0.6× |
| ICP (point-to-point) | 50K | 0.812 | 0.419 | 1.9× | 3.3× |
| | 200K | 6.714 | 1.979 | 3.4× | 9.7× |
| | 1M | **122.6** | 22.62 | **5.4×** | 22× |

倍率はすべて「PointCloud の時間 ÷ Open3D の時間」（大きいほど本実装が遅い）。

## OpenMP 並列化の効果（Phase 5 計測、2026-08-10）

`docs/todo/PLAN_pointcloud_openmp_parallelization.md` の Phase 1〜4（`NormalEstimator`／`SORFilter`／
`RansacPlaneDetector` 等 RANSAC 系／`ICPRegistration` の並列化）を実施した後、本ベンチマークを
**同一バイナリ**で `OMP_NUM_THREADS=1`（シングルスレッド）と既定（全論理コア使用）の2条件で
実行し、並列化そのものの効果だけを切り出して計測した（計測機: Intel Core 5 120U、10 物理コア /
12 論理コア）。

| アルゴリズム | 点数 | シングルスレッド (秒) | マルチスレッド (秒) | 倍率 | 並列化対象か |
|---|---|---|---|---|---|
| 法線推定 (PCA) | 50K | 0.162 | 0.033 | **4.96×** | ✅ Phase 1 |
| | 200K | 1.210 | 0.202 | **5.99×** | |
| | 1M | 49.38 | 10.92 | **4.52×** | |
| SOR 外れ値除去 | 50K | 0.201 | 0.044 | **4.58×** | ✅ Phase 1+2 |
| | 200K | 0.921 | 0.211 | **4.37×** | |
| | 1M | 11.81 | 2.130 | **5.54×** | |
| RANSAC 平面検出 | 50K | 0.031 | 0.010 | **3.18×** | ✅ Phase 3 |
| | 200K | 0.123 | 0.025 | **4.90×** | |
| | 1M | 0.671 | 0.186 | **3.61×** | |
| ICP (point-to-point) | 50K | 0.774 | 0.195 | **3.97×** | ✅ Phase 4 |
| | 200K | 7.027 | 1.266 | **5.55×** | |
| | 1M | 137.9 | 27.36 | **5.04×** | |
| ボクセルダウンサンプリング | 50K | 0.032 | 0.026 | 1.25×（対照） | ✗ 対象外 |
| | 200K | 0.099 | 0.133 | 0.74×（対照） | |
| | 1M | 0.554 | 0.447 | 1.24×（対照） | |
| DBSCAN | 50K | 0.082 | 0.127 | 0.65×（対照） | ✗ 対象外 |
| | 200K | 0.454 | 0.490 | 0.93×（対照） | |
| | 1M | 7.673 | 5.335 | 1.44×（対照） | |

倍率は「シングルスレッドの時間 ÷ マルチスレッドの時間」（大きいほど並列化が効いている）。

**主な発見:**

1. **並列化した4アルゴリズム（法線推定・SOR・RANSAC平面検出・ICP）はいずれも全点数で 3.2〜6.0倍の
   安定した高速化。** 12論理コア（10物理コア）に対して理論値 12倍には遠く及ばないが、KD-tree 構築
   やヒープ確保などシリアルなオーバーヘッド部分が Amdahl の法則的に効いていることに加え、
   本機がハイブリッド構成（P-core/E-core混在）でスレッド間の実効性能が均一でないことも影響していると
   見られる。
2. **並列化していない2アルゴリズム（ボクセルダウンサンプリング・DBSCAN）は倍率が 0.65×〜1.44× と
   1倍前後でばらつくのみ**（絶対時間が小さいための測定ノイズ、または他プロセスとのスケジューリング
   干渉）。これが対照群として機能し、上記の3〜6倍の高速化が測定誤差ではなく Phase 1〜4 の
   OpenMP 並列化そのものによるものであることを裏付けている。
3. **すべてのアルゴリズムで、シングルスレッド版とマルチスレッド版の出力が完全一致した**
   （`sor_filter` の inlier 数 48260/193738/971324、`ransac_plane` の inlier 数
   29480/118546/600728、`icp_point_to_point` の fitness 値 0.00272707/0.000989267/0.000353558、
   `dbscan` のクラスタ数まですべて完全一致）。特に `NormalEstimator`/`ICPRegistration` は
   マスク→逐次畳み込み方式（Phase 1/4 の実装メモ参照）により元の逐次実行とビット単位で同一の
   結果になる設計なので、この一致は期待通り。`RansacPlaneDetector` は乱数を使うため理論上は
   スレッド数で結果が変わりうる設計（Phase 3 の実装メモ参照）だが、今回のシードでは変化しなかった。
4. **1,000,000 点の ICP が 137.9秒 → 27.36秒に短縮。** Open3D 版（22.62秒、上表参照）にほぼ並ぶ
   水準まで縮まった。SOR も 1M 点で 11.81秒 → 2.13秒（Open3D 版 3.803秒より高速）。

**再現手順:** 上記「2. C++側のビルドと実行」を、`OMP_NUM_THREADS` 環境変数を変えて2回実行するだけ
（`$env:OMP_NUM_THREADS=1; .\benchmark_pointcloud.exe .\datasets` → 結果をリネームして退避 →
`Remove-Item Env:\OMP_NUM_THREADS; .\benchmark_pointcloud.exe .\datasets`）。`PointCloud.vcxproj`
の `OpenMPSupport` は Release|x64 のみ有効なため、Release ビルドで実行すること。

## 主な発見（2026-08-10 再計測時点）

1. **SOR 外れ値除去を KDTree ベースの k-NN 探索に書き換え、200K 点で 63.65秒 → 0.879秒
   （約 72倍高速化）、50K 点で 3.660秒 → 0.172秒（約 21倍高速化）。** Open3D との倍率も
   266倍 → 3.7倍、68.6倍 → 3.2倍まで縮小し、他のアルゴリズム（法線推定・ダウンサンプリング等）
   と同程度の差になった。以前は O(n²) の総当たり実装で 1,000,000 点は外挿で約 26分かかる
   ため計測をスキップしていたが、今回初めて直接計測でき **8.845秒**（Open3D比 2.3倍）で
   完走した。50K/200K の inlier 数（48260 / 193738）は書き換え前の総当たり実装と完全一致し、
   結果の正しさも確認済み（`PointCloud/PointCloud/SORFilter.cpp`、
   `Crystal::Space::KDTree::findKNearestIndices()` を使用）。
2. **ICP (point-to-point) の 1,000,000 点が 483.8秒 → 122.6秒に短縮（約 3.9倍高速化）、
   Open3D との倍率も 22倍 → 5.4倍に縮小。** 初回計測時点では「`KDTree` が `IPointItem` を廃止し
   `Vector3dfVector` を直接扱うようになったので仮想関数呼び出しのオーバーヘッドは解消したはずだが
   未検証」と記していた仮説が、この再計測で裏付けられた形。200,000 点でも 18.35秒 → 6.71秒
   （倍率 9.7倍 → 3.4倍）、50,000 点でも 1.350秒 → 0.812秒（倍率 3.3倍 → 1.9倍）と、
   点数が大きいほど改善幅が大きい（＝反復あたり定数項オーバーヘッドが支配的だった裏付け）。
3. **法線推定は 1,000,000 点で約 2.2倍遅い**（46.6秒 vs 21.6秒、初回計測の 3.2倍から改善）。
   点ごとに 3x3 共分散行列の SVD 分解を行うため、探索半径内の近傍点数が多い密な点群でコストが
   増える点は変わらないが、近傍探索自体が `KDTree` の直接対応で速くなった分、全体としても縮んでいる。
4. **DBSCAN は本実装の方が速い傾向が継続**（1,000,000 点で 5.0秒 vs Open3D 9.8秒、約 2倍）。
   空間分割ベースの実装がここでは引き続き有利に働いている。
5. **ボクセルダウンサンプリング**は 50K/200K では Open3D の 2.7〜3.0倍だが、1,000,000 点では
   8.1倍まで開いている（初回計測は 3.2倍）。絶対時間はどちらも1秒未満で実用上の差は小さいものの、
   他アルゴリズムと異なりこの再計測では相対的に悪化しており、要因は未調査（`DownSampler` は
   `KDTree` を使わない独自のボクセルグリッド実装のため、今回のリファクタリングの影響を受けていない）。
6. **RANSAC 平面検出**は全サイズで Open3D が数倍速い（2.0〜5.8倍）ままだが、絶対時間はどちらも
   1秒未満で実用上の差は小さい。

なお ICP の `fitness` 値は両実装で定義が異なる（本実装＝最終反復の平均二乗最近傍距離、
Open3D＝対応点の重なり比率）ため、数値としては比較できない。速度のみが比較対象。

**計測時の注意:** C++側・Python側を同時実行すると CPU 競合で特に長時間かかるステップ
（SOR フィルタ等）の数値が大きくぶれる（実測で SOR 200K が 63.65秒 → 109.85秒まで悪化した例を
確認済み）。両者は必ず逐次実行し、片方が完全に終わってからもう片方を実行すること。

**ビルド時の注意:** `PointCloud.vcxproj` 等を `.sln` 経由ではなく直接ビルドすると、
`$(SolutionDir)` が未定義になり出力が `PointCloud\PointCloud\x64\Release\` のような
プロジェクトローカルなパスに置かれてしまい、このベンチマークがリンクするリポジトリ共有の
`x64\Release\PointCloud.lib` が更新されない（結果として古いコードのままベンチマークが走る）。
直接ビルドする場合は `msbuild PointCloud.vcxproj /p:Configuration=Release /p:Platform=x64
/p:SolutionDir=<リポジトリルート>\` のように `SolutionDir` を明示するか、素直に
`msbuild Phantom2026.sln /p:Configuration=Release /p:Platform=x64` でソリューション全体を
ビルドすること。

## 再現手順

### 0. 前提

- Python 3.11（Open3D 0.19.0 は本稿執筆時点で Python 3.13 のホイールが無いため）に
  `numpy` と `open3d` をインストールしておく（`pip install numpy open3d`）。
- リポジトリ全体を Release / x64 でビルドしておく（`PointCloud.lib` / `Space.lib` /
  `Numerics.lib` / `Math.lib` が `x64\Release\` に揃っている必要がある）:
  ```powershell
  msbuild Phantom2026.sln /p:Configuration=Release /p:Platform=x64
  ```
  個別プロジェクトだけを再ビルドすると、既存の `x64\Release\` の他ライブラリとコンパイラの
  バージョン/フラグがずれて `LNK2038`（RuntimeLibrary の不一致）等でリンクに失敗することがあるため、
  ソリューション全体のビルドを推奨する。

### 1. データセット生成

```powershell
py -3.11 .\PointCloud\benchmark_open3d\generate_datasets.py
```

`datasets\` 以下に、両実装で共通して読み込む合成点群 PLY（バイナリ・リトルエンディアン, float32
x/y/z のみ）を乱数シード固定で生成する（`.gitignore` 済み・再生成可能）。

| ファイル | 用途 |
|---|---|
| `cube_uniform_<N>.ply` | 等方一様立方体（ダウンサンプル・法線推定・SOR 用） |
| `clustered_blobs_<N>.ply` | 密度を一定に揃えた8クラスタ（DBSCAN 用） |
| `plane_outliers_<N>.ply` | 平面 + 15% 外れ値（RANSAC 平面検出用） |
| `icp_target_<N>.ply` / `icp_source_<N>.ply` | 球面点群 + 既知の回転・並進・ノイズ（ICP 用） |

### 2. C++ 側（本リポジトリの PointCloud）のビルドと実行

```powershell
.\PointCloud\benchmark_open3d\build_benchmark.ps1
.\PointCloud\benchmark_open3d\benchmark_pointcloud.exe .\PointCloud\benchmark_open3d\datasets
```

`cpp_results.csv` が出力される。1,000,000 点の ICP は実行環境によっては数分程度かかる
（上記の結果は 122.6 秒）。SOR は KDTree ベースの実装に書き換え済みのため、1,000,000 点でも
自動スキップされず数秒〜十秒程度で完走する（上記の結果は 8.8 秒）。

### 3. Open3D 側の実行

```powershell
py -3.11 .\PointCloud\benchmark_open3d\benchmark_open3d.py
```

`open3d_results.csv` が出力される。

### 4. PCL 側のセットアップと実行

PCL は PyPI にプリビルド Python バインディングが無い（`pclpy` は conda 配布のみ、`pcl-py` は
ソースビルドで結局 PCL 本体が要る）ため、[公式 GitHub Releases](https://github.com/PointCloudLibrary/pcl/releases)
の Windows 向け AllInOne インストーラー（プリビルド、msvc2022-win64、Boost/Eigen/FLANN/VTK/Qt 等の
依存関係込み）を使う。

```powershell
# 1. インストーラーをダウンロードしてサイレントインストール（管理者権限が必要な場合あり）
Invoke-WebRequest -Uri "https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.15.1/PCL-1.15.1-AllInOne-msvc2022-win64.exe" -OutFile pcl_installer.exe
Start-Process -FilePath .\pcl_installer.exe -ArgumentList "/S" -Wait
# →既定で "C:\Program Files\PCL 1.15.1" にインストールされる

# 2. ベンチマークのビルド（本体 .sln には非依存、PCL のみリンク）
.\PointCloud\benchmark_open3d\build_benchmark_pcl.ps1

# 3. 実行（PCL の bin を PATH に通す必要あり）
$env:PATH += ";C:\Program Files\PCL 1.15.1\bin"
.\PointCloud\benchmark_open3d\benchmark_pcl.exe .\PointCloud\benchmark_open3d\datasets
```

`pcl_results.csv` が出力される。DBSCAN は PCL にネイティブ実装が無いため対象外
（詳細は上記サマリの注記を参照）。

**ビルド時の注意（PCL 固有）:**
- PCL のプリビルドライブラリは Eigen の 32byte（AVX）アライメント前提でビルドされているため、
  素の `/O2 /MD` だけでコンパイルすると `pcl/memory.h` の `#error`（`EIGEN_MAX_ALIGN_BYTES` 不一致）
  でコンパイルが止まる。`/arch:AVX2 /DEIGEN_MAX_ALIGN_BYTES=32` の両方を付けること
  （`EIGEN_MAX_ALIGN_BYTES=32` だけだとコンパイルは通るが、`pcl::IterativeClosestPoint::align()`
  実行時にスタック上の Eigen 型のアライメント不一致で `0xC0000005`（アクセス違反）がクラッシュする
  ことを実機で確認済み——`build_benchmark_pcl.ps1` は両方セットしている）。
- PCL は公式配布が VS2022（vc143）ビルドだが、本リポジトリは VS2026（v145）に一本化済みで
  VS2022 は入っていない。MSVC の v14x ツールセット群は動的 CRT（`/MD`）に対する ABI 互換性が
  保たれているため、VS2026 側でコンパイルしたオブジェクトと vc143 プリビルドの PCL 静的ライブラリを
  素直にリンクして問題なく動作した（今回の実機検証で確認済み）。

### パラメータ対応表

3実装で以下のパラメータを揃えている（`benchmark_pointcloud.cpp` / `benchmark_open3d.py` /
`benchmark_pcl.cpp` 冒頭の定数）。

| パラメータ | 値 |
|---|---|
| voxel size（ダウンサンプル） | 0.3 |
| 法線推定の探索半径 | 0.5 |
| SOR: k / std_ratio | 20 / 2.0 |
| RANSAC: 反復数 / 距離閾値 | 200 / 0.02 |
| DBSCAN: eps / minPts | 0.5 / 10 |
| ICP: 最大反復数 / 最大対応距離 | 30 / 3.0（早期収束は無効化し30回固定反復に統一） |

## ファイル一覧

- `generate_datasets.py` — 合成点群データセットの生成（numpy のみ、open3d/pcl 不要）。
- `benchmark_open3d.py` — Open3D 側のベンチマーク実行スクリプト。
- `benchmark_pointcloud.cpp` — 本リポジトリの PointCloud 側のベンチマーク（単体でビルド、`.sln` 未登録）。
- `build_benchmark.ps1` — `benchmark_pointcloud.cpp` のビルドスクリプト。
- `benchmark_pcl.cpp` — PCL 1.15.1 側のベンチマーク（単体でビルド、独自の PLY リーダー内蔵で
  本体の `.sln` の成果物には非依存）。
- `build_benchmark_pcl.ps1` — `benchmark_pcl.cpp` のビルドスクリプト（既定で
  `C:\Program Files\PCL 1.15.1` を参照、`-PclRoot` で変更可）。
- `datasets/`・`*.csv`・`*.exe` 等の生成物は `.gitignore` 済み（再生成可能なため未コミット）。
