# ロボティクス数学学習

[English](README.md)

ロボティクスの背景にある数学を、説明・コード・可視化・検索練習を通して
再現可能な形で学ぶための小さなLabです。

このリポジトリは時系列の勉強日記ではありません。それぞれのトピックを、
他の学習者も実行し、確認し、説明できるArtifactにすることを目指します。

## 学習経路

1. 三角関数と単位円
2. ベクトルと座標成分
3. 回転行列
4. 1リンクアームの順運動学
5. ローカル座標系とWorld座標系
6. 複数Originと2リンクアームの順運動学

## 最初のArtifact：1リンク平面アーム

長さ $L$ のリンクと関節角 $\theta$ に対し、手先位置は次のようになります。

$$
x = L\cos(\theta), \qquad y = L\sin(\theta).
$$

同梱のPythonモジュールは、この手先位置を計算してMatplotlibでアームを可視化します。
0〜360°のSliderを動かすと、アーム、先端、成分補助線、数値がリアルタイムで更新
されます。ボタンから30°、45°、90°へ直接切り替えられます。

```text
角度 θ -> sin/cosによる成分分解 -> 手先位置 (x, y) -> 可視化
```

## クイックスタート

Python 3.11以降を推奨します。

```bash
python -m venv .venv
python -m pip install -e .
python -m robotics_math_study.one_link_arm
python examples/verify_one_link_angles.py
python -m unittest discover -s tests
```

依存関係を分離する場合は、インストール前に仮想環境を有効化してください。
PowerShellでは `.venv\Scripts\Activate.ps1`、macOSまたはLinuxでは
`source .venv/bin/activate` を実行します。

最初のコマンドは対話的なMatplotlibウィンドウを開きます。数値確認スクリプトは
30°、45°、90°の先端座標を表示するため、図と比較できます。

対話的なJupyter Notebookを使う場合は、次を実行します。

```bash
python -m pip install -e ".[notebook]"
jupyter lab notebooks/01_one_link_forward_kinematics.ipynb
```

Notebookでは、ボタンに加えて `visualizer.set_angle(30)`、
`visualizer.set_angle(45)`、`visualizer.set_angle(90)` のようにコードから
代表角度へ切り替えられます。

## 順運動学に相当する場所

`forward_kinematics_1link()`が順運動学の計算です。既知のリンク長と関節角を
受け取ります。それらを`L*cos(θ)`と`L*sin(θ)`で先端座標`(x, y)`へ変換します。
Matplotlib側はその結果を可視化し、角度変更時に描画要素を更新しているだけです。

## 2つ目のArtifact：ローカル座標系と世界座標系

座標系の学習例では、世界座標軸`X, Y`を固定したまま、ローカル座標軸`x', y'`が
アームと一緒に回転します。ローカル基底点を`P_world = R(theta) @ P_local`で変換し、
ローカル座標と世界座標の両方を表示します。

```bash
python examples/visualize_local_world_frames.py
```

Sliderと代表角度ボタンで0°、30°、45°、90°を確認できます。赤い点はローカル
`(1, 0)`の世界座標での移動先で、回転行列の第1列です。緑の点はローカル
`(0, 1)`の移動先で、回転行列の第2列です。

同じ学習例をNotebookでも実行できます。

```bash
jupyter lab notebooks/02_local_and_world_frames.ipynb
```

## 3つ目のArtifact：2リンクアームの複数Origin

2リンクの学習例では、固定されたWorld座標系と、肩・肘のローカル座標系を比較します。
肘は自分の座標系では常に`(0, 0)`ですが、World上の位置は`theta1`によって変わります。
Worldから見た前腕の角度は、相対角度の和`theta1 + theta2`です。

```bash
python -m robotics_math_study.two_link_frames
jupyter lab notebooks/03_multiple_origins_and_frames.ipynb
```

2本のSliderは、両リンク、両ローカル座標系、肘・手首のWorld座標、2リンク順運動学の
ベクトル加算を更新します。Case A〜Dから、Notebookの確認問題で使う代表的な角度の
組み合わせへ切り替えられます。

## リポジトリ構成

```text
robotics-math-study/
├── examples/                  # 数値確認と対話的な学習例
├── notebooks/                 # 対話的で実行可能な実験
├── notes/                     # 自分の言葉で書いた説明
├── src/robotics_math_study/   # 順運動学と可視化のモジュール
└── tests/                     # 数学的な振る舞いの小さな検証
```

## 公開方針

自作の説明、コード、可視化、演習、振り返りは公開対象にできます。一方で、
書籍・教材の転載、授業の課題文や提出解答、会社の非公開情報、顧客情報、
認証情報、個人情報、それらを含む生成物はコミットしません。

授業や書籍をきっかけに扱う題材は、保護された内容を転載せず、概念を独立した
形で自分の言葉に直し、出典を記録します。

## 学習方法

各トピックで次を行います。

1. 考え方を平易な言葉で説明する。
2. 代表的な演習を少数解く。
3. 実装または可視化する。
4. テストまたは検索練習用の問いを加える。
5. 出典と未解決の疑問を記録する。

## ライセンス

ライセンスはまだ選択していません。後からライセンスを追加するまでは、著作権は
リポジトリ所有者に留保されます。
