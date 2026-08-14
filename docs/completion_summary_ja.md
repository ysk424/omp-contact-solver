# omp-contact-solver 完成まとめ

## 目的と完成範囲

`omp-contact-solver` は、GPUを使わずにCPUとOpenMPだけで布を計算する、
独立した軽量ソルバーです。`ppf-contact-solver` やCUDAには依存しません。
ソルバー本体は描画せず、頂点位置だけを返すDLLです。Blenderでの表示と
アニメーション保存は同梱のExtensionが担当します。

完成範囲は次のとおりです。

- 動的な三角形 `SHELL` 1枚
- トポロジーを維持して変形する三角形 `STATIC` 1枚
- OpenMPによる予測、制約射影、PCG、BVH問い合わせ・refitの並列化
- Projective Dynamicsによる伸び・簡易曲げ
- 有限剛性のシームを全体方程式に組み込んだ縫製
- 三角形変形勾配の最大特異値を制限するStrain Limit
- moving STATIC、摩擦、厚みを含むSHELL頂点対STATIC三角形接触
- C ABIを持つWindows DLL
- Blender Extensionによる準備、ベイク、Shape Key出力
- 自動テストとブラウザーで確認できる可視化レポート

PIN、GPU、レンダー、自己衝突、SHELL同士の衝突、四面体、剛体は意図的に
対象外です。

## ソルバー構成

布の一意な三角形エッジをStretch制約、内側エッジの対向頂点間を軽量な
Bend制約として扱います。ローカル射影と全体解を反復し、全体方程式は
matrix-free PCGで解きます。

シームは頂点の強制移動や後処理ではありません。準備時の有限な縫い距離と
剛性を持つ制約として、Stretch、Strain Limit、接触と同じPD全体方程式に
入ります。このため、強制シーム補正で発生した布全体の収縮を避けられます。

Strain Limitは各三角形の3x2変形勾配をSVD相当の計算で射影し、scaled ADMM
dualを通じて全体方程式へ戻します。`5%` はローカル射影の目標
`sigma_max <= 1.05` を意味します。有限回のADMM反復なので、公開結果の全三角形が
常に厳密な5%以内になるハードクランプではありません。

STATICはmedian-split BVHに格納します。アニメーション頂点をサブステップ間で
補間し、OpenMPでBVHをrefitします。接触は両面のclosest-pointと頂点軌跡の
segment-triangle判定から有限な接触ターゲットを作り、PD全体解へ入れます。

## Blenderワークフロー

Extensionの **Prepare Simulation Copies** は、元のオブジェクトを変更せず、
`OMP Contact Simulation` コレクションへ次を作ります。

- ベイク対象の `SHELL` コピー
- Armature、Mesh Cacheなどの変形モディファイアを維持した `STATIC` コピー
- world Z=0.40～1.45 mを既定範囲とする、トポロジー安定なMask crop
- 近接する境界頂点から検出した有限剛性シーム

ボディの切断面は閉じません。接触判定が両面で、parity判定を使わないためです。
実キャラクターでは、crop後のボディは121,746頂点、243,176三角形、微小三角形を
除いた226,002枚が衝突に使われました。従来の約423,534枚から約47%削減しています。

BakeはフレームごとにSTATICを評価してDLLへ送り、結果をabsolute Shape Keysへ
保存します。元のSHELL、STATIC、既存Shape Keysは上書きしません。

## 実キャラクター検証

検証対象はZOZO用の実キャラクターボディと6,563頂点の服です。厚み10 mmでは
初期状態から布頂点の41.49%が接触帯に入り、首や胴を押し広げる原因になりました。
厚み2 mmでは初期接触帯の頂点は0でした。このためBlender側の既定厚みは2 mmです。

フレーム2の指定貫通位置では、Substeps 4で布三角形の3頂点がボディ内部へ
2.26～2.63 mm入りました。同じ入力を再計算すると、Substeps 8で3頂点とも
外側2.26～2.37 mmとなり、貫通は消えました。Substepsの製品既定値は余裕を持たせて
10にしています。

首周りは「滑って広く見える」だけではなく、境界エッジが実際に伸びていました。
フレーム40までの比較結果は次のとおりです。

| Substeps | Stretch | PD反復 | 首周長 | 首幅 | 首境界エッジ95% |
|---:|---:|---:|---:|---:|---:|
| 8 | 5,000 | 8 | +10.7% | +12.9% | +15.4% |
| 10 | 50,000 | 8 | +8.7% | +7.9% | +12.9% |
| 10 | 100,000 | 8 | +7.7% | +6.4% | +11.9% |
| 10 | 100,000 | 16 | +5.2% | +3.5% | +7.9% |

実キャラクター向けの確認済み設定は次です。

| 項目 | 値 |
|---|---:|
| Thickness | 0.002 m |
| Substeps | 10 |
| Stretch Stiffness | 100,000 |
| PD Iterations | 16 |
| PCG Iterations | 64 |
| Strain Limit | 5% |
| Strain Limit Weight | 100,000 |
| Seam Stiffness | 100,000 |
| Collision Safety Passes | 0 |

PCG残差は約`1e-5`まで収束しており、首伸びにはPCG回数よりStretch剛性と
PD/ADMM反復数が効きました。PD 16では上側シームの95%開きも約2.6 mmから
約1.1 mmへ減りました。

## テストと配布物

Releaseビルドでは次を確認しています。

- C++ソルバーテスト
- C ABIテスト（OpenMP、ABI、既定Substeps 10を含む）
- PD、Strain Limit、STATIC接触、animated STATIC、swept contactの可視化テスト
- Blender Extensionの準備、crop、animated STATIC、シーム、ベイクsmoke test

主なコマンドは次です。

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
cmake --build build --config Release --target visual-test-report
cmake --build build --config Release --target blender-extension-test
cmake --build build --config Release --target blender-extension
```

完成版Extensionは
`build/packages/omp_contact_solver-0.5.1-windows-x64.zip` です。

## 既知の限界と将来候補

- moving STATICは各サブステップ位置で判定しますが、動く三角形そのものを含む
  完全な相対CCDではありません。薄い接触や高速運動ではSubstepsを増やします。
- Strain Limitは有限剛性・有限反復です。より厳密にする場合は、違反量を見て
  PD/ADMM反復を追加する適応収束が第一候補です。
- Contact WeightとStrain Limit Weightは現在一部連動しています。将来は独立化し、
  接触も片側ADMM制約として解くと調整しやすくなります。
- 接触はSHELL頂点対STATIC三角形です。edge-edge、自己衝突、完全なcloth CCDは
  実装していません。
- 実物に首リブや伸び止めテープがある服は、境界用の補強材料・制約を追加する方が
  物理的に正確です。

## 完成状態

本リポジトリは、GPUなし・レンダーなし・SHELL/STATIC限定という当初の範囲で、
DLL、OpenMP並列ソルバー、Blender Extension、実キャラクター検証、可視化テスト、
インストールZIPまでを一通り完成しています。ライセンスはMITです。
