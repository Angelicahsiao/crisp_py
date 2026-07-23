![crisp_py_logo](https://github.com/user-attachments/assets/374ae11a-4d82-4bb7-8b93-152bde13aa5b)

[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
![MIT Badge](https://img.shields.io/badge/MIT-License-blue?style=flat)
<a href="https://utiasDSL.github.io/crisp_controllers/"><img alt="Static Badge" src="https://img.shields.io/badge/docs-passing-blue?style=flat&link=https%3A%2F%2FutiasDSL.github.io%2Fcrisp_controllers%2F"></a>
<a href="https://github.com/utiasDSL/crisp_py/actions/workflows/ruff_ci.yml"><img src="https://github.com/utiasDSL/crisp_py/actions/workflows/ruff_ci.yml/badge.svg"/></a>
<a href="https://github.com/utiasDSL/crisp_py/actions/workflows/pixi_ci.yml"><img src="https://github.com/utiasDSL/crisp_py/actions/workflows/pixi_ci.yml/badge.svg"/></a>
<a href="https://utiasDSL.github.io/crisp_controllers#citing"><img alt="Static Badge" src="https://img.shields.io/badge/arxiv-cite-b31b1b?style=flat"></a>

*CRISP_PY /krɪspi/*, a python package to interface with robots using [CRISP controllers](https://github.com/utiasDSL/crisp_controllers). Check the [project website](https://utiasdsl.github.io/crisp_controllers/) for further information!

![crisp_py](https://github.com/user-attachments/assets/e4cbf5fd-6ba7-4d7c-917a-bbb78d79ab10)

## Running in Docker

A pixi-based dev container is provided (`Dockerfile` + `docker-compose.yml`),
mirroring the crisp_gym setup. It ships only the pixi runtime and system
libraries; the repo is **bind-mounted** at runtime and the pixi environment is
created on first start with `pixi install -e humble`, so edits on the host take
effect immediately.

**Expected layout** — the compose file mounts `~/workspace`, so clone this repo
(and any sibling repos you co-develop) under it:

```
~/workspace/
├── crisp_py/     # this repo
└── crisp_gym/    # optional
```

**Build and start:**

```bash
cd ~/workspace/crisp_py
NETWORK_INTERFACE=enp0s31f6 docker compose up -d crisp-py   # set to your NIC (see `ip addr`)
docker exec -it crisp-py-humble bash
```

The first start runs `pixi install -e humble` (a few minutes); later starts
reuse the `.pixi/envs` cache on the host. Inside the container, activate and use
the environment as usual:

```bash
pixi shell -e humble          # ROS 2 Humble + crisp_py
python examples/<...>.py
```

**Notes:**

- The container uses `network_mode: host` (ROS 2 DDS discovery) and mounts
  `/dev` + the X11 socket so visualization tools (viser, yourdfpy) and USB
  devices work.
- `ROS_DOMAIN_ID` is set to `1` in the compose file to match crisp_gym. crisp_py's
  pixi activation (`scripts/set_ros_env.sh`) otherwise defaults it to `100` when
  `scripts/personal_ros_env.sh` is absent — create that file to override.
- For the Jazzy stack, swap `humble` for `jazzy` in the compose `command` and
  the `.pixi/envs/humble` check (or run `pixi install -e jazzy` manually).
