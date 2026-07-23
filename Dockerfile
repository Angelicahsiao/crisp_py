# crisp_py development image (mirrors crisp_gym/Dockerfile).
#
# This is a DEV image: it ships only the pixi runtime and the system libraries
# the ROS 2 / visualization stack needs. The repo itself is bind-mounted at
# runtime (see docker-compose.yml) and the pixi environment is created on first
# container start with `pixi install -e humble`. That keeps the image small and
# lets you edit crisp_py on the host with changes visible immediately.

FROM ghcr.io/prefix-dev/pixi:0.63.2-jammy

USER root

# X11 + OpenGL runtime libs for the visualization tools crisp_py pulls in
# (viser, yourdfpy, matplotlib). No ffmpeg/libav here — crisp_py has no video
# pipeline (that set lives in crisp_gym for lerobot).
RUN apt-get update && apt-get install -y --no-install-recommends \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libxi6 \
    libxkbcommon-x11-0 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/crisp_py

# Pixi metadata drives dependency resolution. These are shadowed by the
# workspace bind-mount at runtime, but copying them lets `docker build` cache a
# layer and (optionally) pre-resolve the env if you uncomment the install below.
COPY pixi.toml pixi.lock* ./

# Optional: bake the environment into the image instead of installing at
# container start. Leave commented to match the crisp_gym dev-mount workflow.
# COPY pyproject.toml ./
# COPY crisp_py ./crisp_py
# COPY scripts ./scripts
# RUN pixi install -e humble

# Safe interactive entrypoint (same as crisp_gym).
ENTRYPOINT ["/bin/bash", "-c", "exec bash"]
CMD []
