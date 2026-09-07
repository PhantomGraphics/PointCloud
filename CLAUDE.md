# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

点群処理（フィルタリング・クラスタリング・法線/曲率推定・平面/円柱検出・メッシュ化）と Gaussian Splatting（GS）を扱うモジュール群。
`PointCloud`（コアライブラリ）、`PointCloudTest`（GoogleTest）、`PointCloudView`（点群処理スタンドアロン ImGui + Vulkan ビューア）、`PointRenderer`（点群/GS 描画の共有ライブラリ）、`GSView`（Gaussian Splatting スタンドアロン ImGui + Vulkan ビューア）の 5 プロジェクトで構成される。

親リポジトリの CLAUDE.md（`../CLAUDE.md`）にビルド方法・全体アーキテクチャ・命名規則が記載されているのであわせて参照すること。

## Build

CMake が唯一のビルド手段（`.vcxproj` は全削除済み。詳細は親リポジトリ `../CLAUDE.md` の Build 節を参照）。

```powershell
# リポジトリルートから、PointCloud単体を設定・ビルド
cmake -S PointCloud -B PointCloud/build_windows -DCMAKE_BUILD_TYPE=Debug
cmake --build PointCloud/build_windows

# または、ルートの CMakePresets.json 経由でリポジトリ全体を一括ビルド
cmake --preset windows-debug
cmake --build --preset windows-debug
```

ターゲット: `PointCloudCore`, `PointCloudTest`, `PointCloudView`, `PointRendererCore`, `GSView`。

依存関係:
- `PointCloud`（コア）は `CGLib/Math`（ヘッダオンリー）に加え `CGLib/Numerics` と `CGLib/Space`（RANSAC・DBSCAN・距離ベースクラスタリングで `Octree`/`KDTree` 等を利用）に依存する。
- `PointRenderer` は点群/GS の Vulkan 描画パイプラインを `PointCloudView` と `GSView` の両方に提供する共有ライブラリ。`CGLib/VulkanGraphics` に依存。
- `PointCloudView` は `CGLib/VulkanGraphics`・`CGLib/UIWidgets`・`CGLib/VkAppBase`・`CGLib/Renderer/VkRenderer`・`PointCloud`・`PointRenderer` に依存。
- `GSView` は `CGLib/VulkanGraphics`・`CGLib/UIWidgets`・`CGLib/VkAppBase`・`CGLib/Volume/VolumeRenderer`・`PointRenderer`・`PointCloud` に依存。

## Tests

```powershell
.\build\windows-debug\PointCloud\PointCloudTest.exe

# フィルター例
.\build\windows-debug\PointCloud\PointCloudTest.exe --gtest_filter=RansacPlaneDetectorTest.*
.\build\windows-debug\PointCloud\PointCloudTest.exe --gtest_filter=DBSCANTest.*
```

183 テストケース（約 31 ファイル）。フィルタ/クラスタリング（`DensityBasedFilterTest`/`DBSCANTest`/`DistanceBasedClusteringTest`/`GroundExtractorTest`/`RegionGrowingTest`）、推定（`NormalEstimatorTest`/`CurvatureEstimatorTest`/`DensityEstimatorTest`/`FPFHEstimatorTest`/`BoundaryDetectorTest`）、検出（`RansacPlaneDetectorTest`/`RansacCylinderDetectorTest`/`RansacSphereDetectorTest`/`RansacConeDetectorTest`）、レジストレーション（`ICPRegistrationTest`/`GlobalRegistrationTest`）、メッシュ化/サーフェス（`GreedyProjectionMeshGeneratorTest`/`PoissonSurfaceTest`/`MLSSurfaceTest`/`ConvexHull2DTest`/`ConcaveHull2DTest`）、ファイル I/O（`PLYFileReaderTest`/`PLYFileWriterTest`/`PCDFileReaderTest`/`PCDFileWriterTest`/`TXTFileReaderTest`/`TXTFileWriterTest`/`PointCloudFileLoaderTest`）、GS（`GSPointCloudTest`）で構成される。近傍探索（k-NN/最近傍）は `Phantom::Space::KDTree` を各アルゴリズムが直接使用し、そのテストは `CGLib/Space/SpaceTest/KDTreeTest.cpp` 側にある。

### シナリオテスト（PointCloudView）

```powershell
.\PointCloud\PointCloudView\run_pc_scenarios.ps1 -Configuration Debug

# 単一シナリオ
.\build\windows-debug\PointCloud\PointCloudView.exe --run-scenario PointCloud\PointCloudView\scenarios\pipeline.json
```

シナリオ JSON は `PointCloud\PointCloudView\scenarios\` にあり、`generate.json`・`file.json`・`filter.json`・`normals.json`・`cluster.json`・`fit.json`・`pipeline.json`・`realworld.json`・`error_handling.json`・`registration.json`・`ground.json`・`surface.json` の 12 本。いずれも `download_pointcloud_samples.ps1` が生成する合成データ（`PointCloud/samples/`）のみで完結し、外部ダウンロードを必要としない
（`realworld.json` は合成生成の `organic_a.ply`/`organic_b.ply` を使う）。

### シナリオテスト（GSView）

```powershell
.\PointCloud\GSView\run_gs_scenarios.ps1 -Configuration Debug
```

シナリオ JSON は `PointCloud\GSView\scenarios\`（`default_state.json`・`load_ply.json`・`load_splat.json`・`load_error.json`・`large_file.json`・`params_sortbased.json`・`params_pbvr.json`・`render_mode.json` の 8 本）で、いずれも合成データのみで完結する
（外部ダウンロード不要。`large_file.json` は合成生成の `gs_large_scene.ply` を使う）。

## Architecture

### Phantom::PC / Phantom::PointCloud（`PointCloud/`）— コアライブラリ

**注意: 名前空間が2種類混在している。** ほとんどのクラス（`PointCloudColoredData`／各種フィルタ・推定器・検出器・ファイル I/O）は `Phantom::PC` 名前空間だが、`GSPointCloud`（Gaussian Splatting 点群）のみ `Phantom::PointCloud` 名前空間になっている。新規クラス追加時はどちらに属するファイルか要確認。

- `PointCloudColoredData`（SoA、`PointCloudFileLoader.h`）— `positions`/`colors`/`normals`/`scalars` を並行配列で保持する主点群コンテナ。`colors`/`normals`/`scalars` は任意（未使用なら空、使う場合は `positions` と同サイズ）で `hasColors()`/`hasNormals()`/`hasScalars()` で判定する。旧 `PointCloud<T>` テンプレート（`std::vector<std::unique_ptr<T>>` による AoS）と `PointXYZ`/`PointXYZf`/`PointXYZRGBf`/`PointXYZRGBNormal` は AoS→SoA 移行で削除済み——利用箇所が `PointCloudTest.cpp` と Python バインディングのみで、コアアルゴリズムはいずれも消費していなかったため。`loadPointCloud()`/`savePointCloud()`（`PointCloudFileLoader.h`）もこの型を直接読み書きする。
- `IPointCloud`（`IPointCloud.h`）— 点群の共通読み書きインタフェース。純粋仮想は `size()`/`getPosition(i)`/`getPositions()`/`setPositions(vector)`/`addPosition(pos)` の5つ。ジェネリックなコード（将来のアルゴリズム・テスト等）が具体的なコンテナ型に依存せず「任意の点群」を読み書きできるようにする拡張の受け皿。`empty()`/`getBoundingBox()` はこのインタフェース上に非仮想メソッドとして一度だけ実装済みで、実装クラス側は再実装不要。`PointCloud`（位置のみ、`PointCloud.h`）と `ColoredPointCloud`（`PointCloudColoredData` を内部に保持するラッパー、`ColoredPointCloud.h`）の2つが実装する。属性の組み合わせごとにクラスを増やす（旧 `PointXYZ*` 系と同じ型爆発）のではなく、色/法線/スカラーは `PointCloudColoredData` 側（`ColoredPointCloud` の `addColor()`/`addNormal()`/`addScalar()`/`getColor()`/`getData()` 等）にまとめて持たせる方針。`feedPositions(consumer, cloud)`（`IPointCloud.h` のフリー関数テンプレート）は `add(const Vector3df&)` を持つ既存アルゴリズム（`NormalEstimator`/`DownSampler`/`CurvatureEstimator`/`DensityEstimator`/`DensityBasedFilter`/`SORFilter`/`RadiusOutlierFilter`/`PassThroughFilter`/`MLSSurface` 等）に `IPointCloud` 経由で点群を流し込む汎用ヘルパーで、`IPointCloud::addPosition()` とは別物（後者は点群自身への追加、前者は他のアルゴリズムへの一括投入）。
- 近傍探索: 専用ラッパーは持たず、`ICPRegistration`/`NormalEstimator`/`RegionGrowing`/`FPFHEstimator`/`BoundaryDetector`/`CurvatureEstimator`/`DensityEstimator`/`DownSampler`/`MLSSurface` の KNN 系メソッドが `Phantom::Space::KDTree` を直接使用する（旧 `PointCloudKdTree` ラッパーは `Space::KDTree` が `Vector3dfVector` を直接扱うようになり実質的な差分がなくなったため削除済み）。既存の `CompactSpaceHash` ベース実装は未移行。
- フィルタ/クラスタリング: `DensityBasedFilter`、`DBSCAN`、`DistanceBasedClustering`、`DownSampler`、`RegionGrowing`(法線・曲率ベースのリージョングロウイング。曲率が低い点から成長を開始し、`Space::KDTree` のk-NN近傍のうち法線角度が閾値内のものを吸収。DBSCAN/距離ベースクラスタリングでは分離できない「同一平面上だが密度的には連続」なケースや、逆に法線が大きく折れ曲がる箇所での過剰結合を防ぐ)。
- 推定: `NormalEstimator`(PCA 法線推定。`orientTowardsViewpoint()` で視点ベースの向き統一が可能)、`CurvatureEstimator`(`estimate()`: PCA固有値比によるスカラー曲率。`estimatePrincipal()`: PCA接平面での局所二次曲面フィッティングにより主曲率 k1/k2 と主方向を推定)、`DensityEstimator`、`FPFHEstimator`(Fast Point Feature Histogram、33次元記述子。法線が既知の点群に対し `Space::KDTree` の k-NN で SPFH を計算し近傍で重み付き合成。大域レジストレーション向けの特徴量基盤)、`BoundaryDetector`(法線既知の点群に対し、接平面に投影した近傍点の角度分布の最大ギャップで境界点を判定)。
- 検出/フィッティング: `RansacPlaneDetector`、`RansacCylinderDetector`、`RansacSphereDetector`(4点から閉形式で球を解く)、`RansacConeDetector`(サンプルのPCA主方向を軸候補とし、軸からの半径が頂点からの距離に比例するというコーン特有の性質を最小二乗直線回帰で利用して頂点・軸・半頂角を推定。半頂角が小さすぎる/大きすぎる劣化フィットは棄却)。
- 地面抽出: `GroundExtractor`(簡易 Progressive Morphological Filter。XYグリッドの最小標高マップに対しウィンドウを段階的に拡大しながらグレースケール開処理を繰り返し、開処理前後の標高差が閾値を超えたセルを非地面と判定)。
- レジストレーション: `ICPRegistration`（`align()`: point-to-point ICP、重み付き Kabsch/Umeyama によるスケール推定オプション付き。`alignPointToPlane()`: 法線ベースの point-to-plane ICP、6元1次方程式を毎反復 Gauss-Jordan 消去で解く Gauss-Newton 線形化。両方とも `RobustKernel`（Huber/Tukey）による対応点の重み付けと `Space::KDTree` による最近傍探索に対応。`computeRigidTransform`（重み付き Kabsch/Umeyama フィット）は `GlobalRegistration` からも使う汎用ユーティリティとして `public static` 公開）、`GlobalRegistration`（FPFH+RANSAC による大域レジストレーション。初期姿勢なしで粗い剛体変換を推定し、`ICPRegistration` による仕上げの前段として使う想定。`FPFHEstimator` の記述子空間での片方向最近傍対応付け→RANSAC で辺長比較による幾何整合性チェック→インライア数最大のモデルをインライア全体で再フィット、という流れ。スケール推定は行わない剛体限定）。
- メッシュ化/サーフェス: `GreedyProjectionMeshGenerator`、`PoissonSurface`、`MLSSurface`(Moving Least Squares。PCA接平面上で重み付き最小二乗フィットした局所二次曲面/平面に点を投影する `smooth()` と、その曲面上をタンジェント平面グリッドでリサンプルして新規点を生成する `upsample()`)、`ConvexHull2D`(XY投影の2D凸包、Andrew's monotone chain)、`ConcaveHull2D`(XY投影の2D凹包。k近傍法(Moreira & Santos 2007)によるアルファシェイプの簡易代替。境界上で最も時計回りに曲がる近傍点を辿り、自己交差または全点包含に失敗したらkを増やして再試行)。
- ファイル I/O: `PLYFileReader`/`PLYFileWriter`（ASCII/バイナリ PLY）、`PCDFileReader`/`PCDFileWriter`、`TXTFileReader`/`TXTFileWriter`、`PointCloudFileLoader::loadPointCloud()`（拡張子から自動判別する統合ローダー）。
- Gaussian Splatting: `GSPoint`（位置・スケール・回転・SH係数・opacity を持つ GS 点）、`GSPointCloud`（`readFromFile()` で GS 用 PLY（標準／SuperSplat 圧縮）または `.splat`（32バイト/スプラットのフラットバイナリ）を拡張子から自動判別して読み込む専用点群。`.ksplat` は仕様が複雑なため未対応）。

### PointRenderer（`PointRenderer/`）— 点群/GS 描画共有ライブラリ

`Phantom::VKR`（`include/`/`src/` 構成、他プロジェクトと異なり `include`/`src` 分離）。`VkPointRenderer`/`VkPointCloudPipeline`/`VkPointScene` が通常点群の描画を、`VkGSPointRenderer`/`GSSplat` が GS スプラットの描画を担当。`PointCloudView` と `GSView` の両方から参照される。

### PointCloudView（`PointCloudView/`）— 点群処理スタンドアロン ImGui + Vulkan アプリ

`PointCloudApp : VkAppBase` 直下。**`VPC` 名前空間**（コアライブラリの `Phantom::PC` とは別）。

- `World`（`World.h`）— `PointCloudfScene` の集合を ID 管理する軽量な状態保持クラス（`addScene()`/`removeScene()`/`findById()`）。ポリゴンオーバーレイ（`PolygonMesh`）も `addPolygon()`/`clearPolygons()` で保持する。
- **GUI 構造（2026-09-07、`docs/todo/PLAN_pointcloudview_gui_restructuring.md` Phase 1〜3）**: PhysicsView と同じ
  「メニューでページを選び、単一の "Control" ウィンドウに表示」方式へ再構築済み。
  - `ControlPanelHost : ::VKG::IVkUIPanel`（`ControlPanelHost.{h,cpp}`）— 唯一の "Control" ウィンドウ。
    `ControlPage`（`Scenes`/`Rendering`/`Processing`/`ImportExport`/`ScenarioBrowser`）を1つだけ表示し、
    上部に共通ステータス（`PointCloudApp::drawStatusArea`：対象シーン選択 Combo・総点数/可視点数・
    描画モード・直近 I/O・シナリオ状態、対象シーンが非表示なら警告）を出す。`page`/`visible`/`process` を
    `pointcloudview_control_layout.ini` に保存。**capture モード**（`--screenshot` またはシナリオ実行、
    `PointCloudApp::setCaptureMode` / `loadScenario()`）ではこの ini・ImGui の `imgui.ini`（`IniFilename=nullptr`）・
    Control ウィンドウ位置（`setFixedLayout`）の 3 つを独立に固定する。`main.cpp` の `--size WxH` /
    `--page N` / `--process N` は検証キャプチャ用（capture モードは ini を読まないため明示指定する）。
  - `PointCloudMenu`（`PointCloudMenu.{h,cpp}`、旧 `Menu`）— 「PointCloud」メニュー。ページと処理を
    *選択するだけ*で World 更新・パネル生成はしない。処理は `ProcessRegistry`（`ProcessId` enum +
    カテゴリ表 Generate/Features/Filters/Segmentation/Fitting/Registration/Surface + `makeProcessView()`）に集約。
  - `ProcessPanel : IEmbeddedPanel`（`ProcessPanel.{h,cpp}`）— Processing ページ。`ProcessId` ごとに
    `IProcessView` を遅延生成し `std::array` で保持（パラメーターがセッション中残る）。Reset で当該パネルのみ破棄。
    `blockReason()` が前提条件不足（点群未ロード／対象シーン未選択／Registration の参照シーン不足）を
    判定し、理由を出して当該パネルを `BeginDisabled()` する（Generate* は空 World でも常時可）。
  - **GUI/シナリオ経路の共通化（Phase 4、完了）**: 処理ロジックは `PointCloudOps.{h,cpp}`（`VPC::ops::`）の
    型付き関数に集約され、`*View`（GUI）と `CommandDispatcher`（シナリオ）が同じ関数を呼ぶ。
    Dispatcher に対応コマンドのある処理は全 20 op 移行済み（DownSample / normals / 各種 filter /
    RANSAC 4 種 / clustering 3 種 / Ground / Curvature / FPFH / Boundary / MLS 2 種 / Hull 2 種 /
    ICP 2 種 / GlobalRegister）。`CommandDispatcher.cpp` はアルゴリズム include をすべて撤去し、
    `Error:<reason>` 整形と `lastMetrics_` 反映だけを行う。`IProcessView::onImGui` の第3引数は
    `std::function<void(int)> onResult`（新 active scene id、-1 = 維持）。`CommandDispatcher` は自前の
    `activeId_` を廃し `renderer_.getActiveSceneIdPtr()` を `setActiveSceneIdRef()` で共有——GUI 選択と
    シナリオ対象が常に一致。**新規処理は `PointCloudOps` に関数を書き、GUI と Dispatcher の両方から呼ぶこと。**
    Dispatcher コマンドを持たない GUI 専用（Density Estimator / Greedy・Poisson メッシュ /
    パラメトリック Generate）は未移行。
  - `ImportExportPanel : IEmbeddedPanel` — Import/Export ページ。I/O は `PointCloudApp` コールバックへ委譲。
  - Scenes/Rendering/ScenarioBrowser ページは既存 `SceneListPanel::onImGui()` /
    `PointCloudRenderer::drawImGuiControls()` / `ScenarioBrowserPanel::drawEmbedded()` を
    `FnEmbeddedPanel`（`IEmbeddedPanel.h`）で包む。
- 各処理パネルは `IProcessView`（`getName()`/`onImGui(World&, activeSceneId, onRebuild)`）を実装する:
  - フィルタ/ダウンサンプル: `DensityBasedFilterView`/`CurvatureBasedFilterView`/`DownSamplerView`
  - 推定: `DensityEstimatorView`/`NormalEstimatorView`（`orientTowardsViewpoint` 対応）/`CurvatureEstimatorView`（スカラー曲率＋主曲率 k1/k2 モード）/`FPFHEstimatorView`/`BoundaryDetectorView`
  - 検出/フィッティング: `RansacPlaneDetectorView`/`RansacCylinderDetectorView`/`RansacSphereDetectorView`/`RansacConeDetectorView`
  - クラスタリング/セグメンテーション: `DBSCANClusteringView`/`DistanceBasedClusteringView`（メニュー表記 "Region Growing (Distance)"）/`RegionGrowingView`（法線・曲率ベース、メニュー表記 "Region Growing (Normal/Curvature)"。既存の距離ベースとクラス・コマンド名（`SegmentRegionGrowing`）を明確に分離）/`GroundExtractorView`
  - レジストレーション: `ICPRegistrationView`（point-to-point / point-to-plane 切替、target シーンを Combo で選択する2点群UI）/`GlobalRegistrationView`（FPFH+RANSAC）
  - メッシュ化/サーフェス: `GreedyProjectionMeshGeneratorView`/`PoissonSurfaceView`/`MLSSurfaceView`（Smooth/Upsample の2ボタン）/`ConvexHull2DView`/`ConcaveHull2DView`
- `SceneListPanel` — シーン一覧 UI。
- `CommandDispatcher : IScenarioDispatcher` — シナリオ用コマンド文字列ディスパッチャ。点群以外のスカラー結果（ICP fitness・hull 面積・inlier 数等）は `lastMetrics_` に格納し `GetLastMetric:<name>` で読み出す。
- 描画: `PointCloudRenderer`/`VulkanPointCloudPipeline`（通常点群）、`VkGSPointRenderer`/`GSPointPresenter`（GS プレビュー、`PointRenderer` を利用）。

**新しいアルゴリズムを追加する場合:** (1) 処理本体を `PointCloudOps.{h,cpp}`（`VPC::ops::`）に型付き関数として書く
（`World& / int activeSceneId / <Params>` を取り `ProcessOutcome` 系を返す）。(2) `IProcessView` を実装したパネルを
追加し、その関数を呼ぶ。(3) `ProcessRegistry.{h,cpp}` に `ProcessId` 定数・`processName()`・カテゴリ表エントリ・
`makeProcessView()` の分岐を足す（メニューは自動生成される）。(4) `CommandDispatcher.cpp` に同じ関数を呼ぶ
シナリオ用コマンドを追加する。ビルドは `PointCloud/CMakeLists.txt` の `file(GLOB)` で新規 `.cpp` を自動的に拾う。

**シェーダー:** `PointCloud/CMakeLists.txt` は `phantom_add_runtime_shaders()`（`../cmake/PhantomVulkanApp.cmake`）で
`PointRenderer/shaders`・`PointCloudView/shaders`・`GSView/shaders` の GLSL を `glslc` で `.spv` 化して exe 横へ置く
（`*.spv` は `.gitignore` 済み。同名は先に渡したディレクトリが優先 = PointCloudView は `PointRenderer/shaders` 優先）。

### GSView（`GSView/`）— Gaussian Splatting スタンドアロン ImGui + Vulkan アプリ

`GSViewApp : VkAppBase` 直下。**`GSView` 名前空間**。ソートベース GS 描画と PBVR（Particle-Based Volume Rendering）風描画の 2 モードに対応。

- `GSComputePBVR` — Vulkan Compute（`gs_pbvr_gen.comp`）による GPU 上での PBVR 用パーティクル生成。シングルパス + alpha=0 カリング方式（CPU 生成から移行済み）。
- `GSParticleGenerator` — パーティクル生成の共通処理。
- `GSViewRenderer` — `PointRenderer` の `VkGSPointRenderer` を用いた GS スプラット描画。
- `GSViewCommandDispatcher : IScenarioDispatcher` — シナリオ用コマンド文字列ディスパッチャ。

## Key Conventions

- **例外禁止**: このリポジトリ全体の規約に従い、`throw`/`try`/`catch` は使わない。エラーは `bool`/`std::optional` で返す（例: `PLYFileReader::readFromFile()`、`PointCloudFileLoader::loadPointCloud()`）。`PointCloudView` の `CommandDispatcher.cpp` はシナリオコマンドの数値引数パースに `std::from_chars` を使う（`std::stof`/`std::stoi` の `try/catch` は使わない）。新規コマンドもこのパターンに倣うこと。
- **名前空間の不統一に注意**: 上記の通り `Phantom::PC`（大半のコアクラス）と `Phantom::PointCloud`（`GSPointCloud`）が混在する。ファイルを開いて確認してから実装すること。
- **非所有ポインタ**: `World::findById()`/`addScene()` が返すシーンポインタは `World` が所有する。呼び出し元は寿命を管理しない。
- **PointRenderer の共有**: 点群/GS 描画パイプラインを変更する場合は `PointCloudView` と `GSView` の両方に影響する。片方だけで検証して終わらせないこと。
