# =================================================================
#            TurtleBot3用 ROS 2 Jazzy 最小構成イメージ
# =================================================================
#
# ■ 概要
#   このDockerfileは、ROS 2 JazzyでTurtleBot3を動作させるための、
#   最小限の「基本構成」環境をビルドします。
#   ロボット上で実行することを目的としています。
#
# ■ 主な構成要素
#   - ベースイメージ: ros:jazzy-ros-base (Ubuntu 24.04)
#   - TurtleBot3パッケージ (ソースビルド, jazzyブランチ):
#     - turtlebot3
#     - turtlebot3_msgs
#     - DynamixelSDK
#
# ■ 含まれないもの と 用途
#   以下のパッケージは、軽量化のため意図的に「含んでいません」。
#   - カメラドライバ
#   - SLAM (Cartographer) および ナビゲーション (Nav2)
#   - GUIツール (RViz, RQT)
#
#   このイメージの用途は、ロボットの基本動作 (bringup) のみです。
#   SLAMやナビゲーションなどの高負荷処理は、リモートPC側で
#   実行することを想定しています。
#
# ■ 使用方法
#   このDockerfileをビルドし、対応する `docker-compose.yml` を
#   使ってコンテナを起動してください。
#   TURTLEBOT3_MODELなどの環境変数は `docker-compose.yml` 側で
#   設定します。
#
# -----------------------------------------------------------------
# 作成者: 菅 匠汰
# 更新日: 2025/10/07
# -----------------------------------------------------------------

# ベースイメージとしてROS 2 Jazzyを選択
FROM ros:jazzy-ros-base

# Dockerビルド時の対話を無効化
ENV DEBIAN_FRONTEND=noninteractive

# TurtleBot3の動作とビルドに最低限必要なパッケージのみインストール
RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ROS 2ワークスペースのセットアップ
WORKDIR /ros2_ws

# TurtleBot3関連のリポジトリを Jazzy ブランチでクローン
RUN git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git src/turtlebot3_msgs
RUN git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git src/DynamixelSDK
RUN git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git src/turtlebot3

# ROSの環境設定を読み込み、依存関係をインストールし、ビルドを実行
# ros-baseイメージではrosdep initが未実行のため `rosdep init || true` を挟んで安定化
RUN . /opt/ros/jazzy/setup.sh && \
    apt-get update && \
    rosdep init || true && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro jazzy -y && \
    colcon build

# コンテナ起動時にROS環境を自動でセットアップするための設定
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# デフォルトのコマンドとしてbashを起動
CMD ["/bin/bash"]