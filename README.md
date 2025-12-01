# ROS 2 Jazzy TurtleBot3 Desktop Docker環境 (for Raspberry Pi 5)

これは、TurtleBot3をROS 2 Jazzyで動作させるためのDocker環境です。

Desktop版のイメージを採用し、GUIツールやカメラ関連パッケージを含めたオールインワン構成になっています。Raspberry Pi 上での動作を想定しています。

## 概要

- **GUI対応**: `ros-jazzy-desktop` をベースにしており、コンテナ内でGUIツール（rqt, rviz2等）を使用可能です（X11転送を使用）。
- **ビルド済みパッケージ**: TurtleBot3関連パッケージをソースビルドではなく `apt` パッケージマネージャでインストールするため、イメージ作成（ビルド）が高速です。
- **カメラ・画像処理**: `v4l2_camera` や `image_transport` を標準搭載しており、カメラを使用した開発がすぐに始められます。
- **ユーザー権限**: セキュリティとGUIの互換性を考慮し、非rootユーザー（user）で動作します。

## 環境対応

- **デバイス**: Raspberry Pi 5 , Raspberry Pi 4
- **ホストOS**: Ubuntu 24.04 LTS (推奨)
- **ROS 2 ディストリビューション**: Jazzy Jalisco

### 主要パッケージ

- `ros-jazzy-turtlebot3`
- `ros-jazzy-desktop` (Rviz2, rqt等を含む)
- `ros-jazzy-v4l2-camera`
- `ros-jazzy-image-transport`

## 必要なもの

- Docker
- Docker Compose V2 (最近のDockerには標準で含まれています)

## Dockerのインストール (Ubuntu 24.04 の場合)

Raspberry Pi 5 (Ubuntu 24.04) にDockerをセットアップする標準的な手順です。

### Docker公式リポジトリのセットアップ

```bash
# 必要なパッケージのインストール
sudo apt update
sudo apt install -y ca-certificates curl

# GPGキーの追加
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# リポジトリの追加
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

### Docker Engineのインストール

```bash
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### ユーザー権限の設定 (推奨)

sudo なしでDockerコマンドを実行できるようにします。

```bash
sudo usermod -aG docker ${USER}
```

※ 設定反映のため、一度ログアウトして再ログインするか、再起動してください。

## 使い方

### 1. リポジトリの準備

```bash
git clone <このリポジトリのURL>
cd <リポジトリディレクトリ>
```

### 2. ワークスペースの作成

ホスト側のディレクトリをコンテナ内にマウントして開発に使用します。

```bash
# ソースコード配置用のディレクトリを作成
mkdir -p ros_ws/src
```

### 3. コンテナのビルドと起動

以下のコマンドでDockerイメージのビルドとコンテナの起動を行います。初回はパッケージのダウンロードが行われるため時間がかかりますが、aptインストールを利用しているため、ソースビルド版より高速です。

```bash
docker compose up -d --build
```

### 4. コンテナへのアクセス

起動したコンテナの中に入ります。サービス名を指定して実行します。

```bash
docker compose exec turtlebot3-jazzy bash
```

### 5. TurtleBot3の起動・動作確認

コンテナ内に入ると、既に環境変数が設定された状態になっています。

**TurtleBot3の起動 (Bringup)**:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**カメラノードの起動 (例)**:

```bash
ros2 run v4l2_camera v4l2_camera_node
```

### GUIアプリケーションの使用 (Rviz2, rqt)

この環境はX11転送を利用して、コンテナ内のGUIアプリをホスト側の画面に表示できます。ホスト側（Raspberry Piに接続したモニタやVNC接続先）で以下を実行してください。

**ホスト側での許可設定**:

```bash
xhost +local:docker
```

※ 毎回行うのが手間の場合は `.bashrc` に記述するか、セキュリティリスクを理解した上で `xhost +` を使用してください。

**コンテナ内での実行**:

```bash
# 例: rqtの起動
rqt

# 例: カメラ映像の確認
ros2 run rqt_image_view rqt_image_view
```

## カスタム開発について

### パッケージの追加

自作パッケージや追加のソースコードは、ホスト側の `ros_ws/src` ディレクトリに置いてください。これらはコンテナ内の `/home/user/ros2_ws/src` に自動的にマウントされます。

### ビルド手順

コンテナ内で以下の通りビルドします。

```bash
# コンテナに入る
docker compose exec turtlebot3-jazzy bash

# ワークスペースへ移動 (ログイン時に自動的にここにいますが念のため)
cd ~/ros2_ws

# 依存関係の解決 (必要な場合)
rosdep install -i --from-path src --rosdistro jazzy -y

# ビルド
colcon build

# 環境の再読み込み
source install/setup.bash
```

## 設定の変更 (compose.yaml)

`compose.yaml` ファイル内の `environment` セクションを編集することで、主要な設定を変更できます。

```yaml
environment:
  - TURTLEBOT3_MODEL=burger   # burger, waffle, waffle_pi
  - LDS_MODEL=LDS-02          # LDS-01, LDS-02
  - ROS_DOMAIN_ID=30          # 通信ID (PC側と合わせる)
```

## ディレクトリ構成

```
.
├── compose.yaml       # コンテナ構成 (旧 docker-compose.yml)
├── Dockerfile         # イメージ設計図
├── README.md          # 本ファイル
└── ros_ws/
    └── src/           # 開発用ソースコード置き場 (ホスト<->コンテナ共有)
```
