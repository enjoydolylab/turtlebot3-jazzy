# =================================================================
#             TurtleBot3 ROS 2 Jazzy Desktop Image
# =================================================================
# Base Image: osrf/ros:jazzy-desktop
# Desktop版は osrf/ros リポジトリで提供されています
FROM osrf/ros:jazzy-desktop

# ビルド時の対話を無効化
ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------------
# 1. 基本ツールとROS 2パッケージのインストール
# -----------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    # 基本ツール
    git \
    python3-rosdep \
    python3-colcon-common-extensions \
    vim \
    wget \
    # USB/Video/Camera Tools
    v4l-utils \
    usbutils \
    # TurtleBot3 Packages (aptでインストール)
    ros-jazzy-turtlebot3 \
    # 追加リクエスト: カメラ・画像関連
    ros-jazzy-v4l2-camera \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------
# 2. ユーザー設定 (GUIアプリケーション用)
# -----------------------------------------------------------------
# ホストOSのユーザーIDと一致させるための引数
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

# 既存のユーザー(ubuntu:1000)がいる場合、競合を防ぐために削除する処理を追加
RUN if id -u $USER_UID >/dev/null 2>&1; then userdel -r $(id -un $USER_UID); fi

# ユーザーを作成し、sudo権限を付与
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# -----------------------------------------------------------------
# 3. 環境設定
# -----------------------------------------------------------------
# 一般ユーザーに切り替え
USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# ROS環境の自動読み込み設定
# エントリポイントではなく .bashrc に追記して永続化
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

CMD ["/bin/bash"]