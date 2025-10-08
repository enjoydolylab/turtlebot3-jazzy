# ROS 2 Jazzy TurtleBot3 最小構成Docker環境

これは、TurtleBot3をROS 2 Jazzyで動作させるための、最小限の基本構成（Bringup）に特化したDocker環境です。ロボット本体のコンピュータ（SBCなど）上で軽量に動作させることを目的としています。

## 概要

このDocker環境は、TurtleBot3の基本的な起動に必要なパッケージのみをソースからビルドして構成されています。SLAMやナビゲーション、GUIツールといったリソースを多く消費する機能は意図的に除外しており、それらの高負荷処理はリモートPC側で実行することを想定しています。

開発時には、ホストPCのソースコードディレクトリをコンテナにマウントすることで、コードの編集からビルド、実行までを効率的に行うことができます。

-----

## 特徴

  - **ROS 2 ディストリビューション**: **Jazzy Jalisco** (Ubuntu 24.04)
  - **ベースイメージ**: `ros:jazzy-ros-base` を使用し、軽量化を実現
  - **主要パッケージ**:
      - `turtlebot3`
      - `turtlebot3_msgs`
      - `DynamixelSDK`
      - `ld08_driver`
  - **開発対応**: `volumes`設定により、ホストPCでのソースコード編集が即座にコンテナへ反映

-----

## 必要なもの

  - [Docker](https://www.docker.com/)
  - [Docker Compose](https://docs.docker.com/compose/install/)

### Dockerのインストール (Ubuntuの場合)

Dockerが未インストールの場合、以下の手順でインストールしてください。

1.  **Docker公式リポジトリをセットアップします。**

    ```bash
    # パッケージリストを更新し、必要なツールをインストール
    sudo apt update
    sudo apt install -y ca-certificates curl gnupg

    # Dockerの公式GPGキーを追加
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg

    # リポジトリを設定
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    ```

2.  **Docker Engineをインストールします。**

    ```bash
    sudo apt update
    sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```

3.  **(推奨) `sudo`なしでDockerを実行する設定**
    毎回`sudo`を付ける手間を省くため、現在のユーザーを`docker`グループに追加します。

    ```bash
    sudo usermod -aG docker ${USER}
    ```

    **設定を反映させるために、一度PCを再起動するか、再ログインしてください。**

-----

## 使い方

#### 1\. ワークスペース用ディレクトリの作成

`docker-compose.yml`で設定しているボリュームマウントのために、ホストPC側にソースコードを配置するディレクトリを作成します。（このプロジェクトのルートディレクトリで実行してください）

```bash
mkdir -p ros_ws/src
```

#### 2\. コンテナのビルドと起動

以下のコマンドを実行すると、`Dockerfile`を基にDockerイメージがビルドされ、コンテナがバックグラウンドで起動します。

```bash
docker compose up --build -d
```

> **Permission Deniedエラーが出る場合**
> 上記コマンドで`permission denied`というエラーが表示された場合は、Dockerの実行権限がありません。以下の`sudo`を付けたコマンドを実行してください。
>
> ```bash
> sudo docker compose up --build -d
> ```
>
> もしくは、前述の「`sudo`なしでDockerを実行する設定」を行うことを推奨します。

#### 3\. コンテナへのアクセス

起動したコンテナの中に入るには、以下のコマンドを実行します。

```bash
docker compose exec turtlebot3 bash
```

コンテナ内では、ROS 2の環境変数が自動的に読み込まれる設定になっています。

#### 4\. TurtleBot3の起動

コンテナ内で以下のコマンドを実行すると、TurtleBot3が起動します。

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

-----

## カスタムパッケージでの開発

この環境を使って独自のROS 2パッケージを開発する手順は以下の通りです。

1.  **ソースコードの配置**:
    開発したいROS 2パッケージを、**ホストPC**の`ros_ws/src`ディレクトリに置きます。

2.  **コンテナ内でビルド**:
    コンテナに入り、ワークスペースのルートで`colcon build`を実行します。

    ```bash
    # コンテナに入る
    docker compose exec turtlebot3 bash

    # ワークスペースに移動してビルド
    cd /ros2_ws
    colcon build
    ```

3.  **環境変数の再読み込み**:
    ビルドしたパッケージをROS 2システムに認識させるため、`setup.bash`を再読み込みします。

    ```bash
    source install/setup.bash
    ```

    これで、`ros2 run`や`ros2 launch`で自作のパッケージを実行できるようになります。

-----

## 設定

TurtleBot3のモデルやROSのドメインIDなどの設定は、`docker-compose.yml`内の`environment`セクションで変更できます。

```yaml
# docker-compose.yml

...
  environment:
    - "TURTLEBOT3_MODEL=burger" # waffle, waffle_pi に変更可能
    - "LDS_MODEL=LDS-02"
    - "ROS_DOMAIN_ID=30"
...
```

-----

## ディレクトリ構成

```
.
├── docker-compose.yml      # コンテナの構成ファイル
├── Dockerfile              # Dockerイメージの設計図
├── README.md               # このファイル
└── ros_ws/
    └── src/                # ⬅ 開発するROSパッケージをここに置く
```