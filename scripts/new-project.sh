#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/new-project.sh --name <repo-name> [--dir <parent-dir>] [--type single|meta]
                         [--gh <owner>] [--private|--public] [--no-push] [--no-git]

Examples:
  scripts/new-project.sh --name mybot-nav --dir ~/Documents/Projects --type single --gh yourname --private
  scripts/new-project.sh --name perception-stack --type meta

Flags:
  --name       Required. New project/repo name (directory will be <dir>/<name>)
  --dir        Parent directory (default: ~/Documents/Projects)
  --type       'single' (default) or 'meta'
  --gh         GitHub owner (creates remote "owner/name" with gh if available)
  --private    Create repo as private (default if --gh is set)
  --public     Create repo as public
  --no-push    Do not push initial commit even if --gh used
  --no-git     Do not init a local git repo
EOF
}

# -------- arg parse --------
name=""
parent="${HOME}/Documents/Projects"
ptype="single"
gh_owner=""
visibility="private"
do_push=1
do_git=1

while [[ $# -gt 0 ]]; do
  case "$1" in
  --name)
    name="${2:-}"
    shift 2
    ;;
  --dir)
    parent="${2:-}"
    shift 2
    ;;
  --type)
    ptype="${2:-}"
    shift 2
    ;;
  --gh)
    gh_owner="${2:-}"
    shift 2
    ;;
  --private)
    visibility="private"
    shift
    ;;
  --public)
    visibility="public"
    shift
    ;;
  --no-push)
    do_push=0
    shift
    ;;
  --no-git)
    do_git=0
    shift
    ;;
  -h | --help)
    usage
    exit 0
    ;;
  *)
    echo "Unknown arg: $1"
    usage
    exit 1
    ;;
  esac
done

[[ -n "$name" ]] || {
  echo "ERROR: --name is required"
  usage
  exit 1
}
[[ "$ptype" == "single" || "$ptype" == "meta" ]] || {
  echo "ERROR: --type must be single|meta"
  exit 1
}

mkdir -p "$parent"
proj_dir="$(realpath -m "$parent/$name")"
[[ ! -e "$proj_dir" ]] || {
  echo "ERROR: path exists: $proj_dir"
  exit 1
}

# -------- scaffold --------
echo "Creating ${ptype} project at: $proj_dir"
mkdir -p "$proj_dir/ws/src" "$proj_dir/scripts"

# .env with a random ROS_DOMAIN_ID
ros_domain_id=$(((RANDOM % 200) + 1))
cat >"$proj_dir/.env" <<EOF
# Per-project environment
ROS_DOMAIN_ID=$ros_domain_id
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# ROS_LOCALHOST_ONLY=1
EOF

# .gitignore (differs slightly by type)
if [[ "$ptype" == "single" ]]; then
  cat >"$proj_dir/.gitignore" <<'EOF'
# colcon outputs
ws/build/
ws/install/
ws/log/
EOF
else
  cat >"$proj_dir/.gitignore" <<'EOF'
# vcstool-managed sources & builds
ws/src/
ws/build/
ws/install/
ws/log/
EOF
  # seed ws.repos (empty skeleton with example)
  cat >"$proj_dir/ws.repos" <<'EOF'
repositories:
  # mybot_description:
  #   type: git
  #   url: git@github.com:yourname/mybot_description.git
  #   version: main
  # mybot_nav:
  #   type: git
  #   url: git@github.com:yourname/mybot_nav.git
  #   version: main
EOF
  : >"$proj_dir/ws.repos.lock"
fi

# pick a readable note for src/ line in README
if [[ "$ptype" == "meta" ]]; then
  src_note="populated by vcstool (from ws.repos)"
else
  src_note="committed in this repo"
fi

# README (expanded & improved)
cat >"$proj_dir/README.md" <<EOF
# $name

**Type:** \`$ptype\`

This project is scaffolded to work seamlessly with the **ros-workspace** infra (Docker + ROS 2 dev environment).
For Docker/infra details and helper scripts like \`workon.sh\`, see: **ros-workspace** → https://github.com/Kyaw-Thiha/ros-workspace

---

## Contents

- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Repository Layout](#repository-layout)
- [Scripts Overview](#scripts-overview)
  - [scripts/sync.sh](#scriptssyncsh)
  - [scripts/freeze.sh](#scriptsfreezesh)
  - [scripts/build.sh](#scriptsbuildsh)
  - [scripts/clean.sh](#scriptscleansh)
  - [scripts/create_pkg.sh](#scriptscreate_pkgsh)
- [Lockfiles & Sources](#lockfiles--sources)
- [Common Tasks](#common-tasks)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

- You are using the Docker-based ROS environment from **ros-workspace** (link above).
- Inside the container you have:
  - ROS 2 (sourced automatically by our scripts).
  - \`colcon\`, \`vcstool\` (for meta projects).
- On host: a checkout of this repo (and the infra repo), plus the ability to mount this project into the container (see \`workon.sh\` in infra).

---

## Quick Start

On the host (outside the container), from your infra repo:

\`\`\`bash
# mount this project into your ROS container
/path/to/ros-workspace/scripts/workon.sh "$proj_dir"
\`\`\`

Once inside the container:

\`\`\`bash
# (Only for multi-repo project) fetch sources defined in ws.repos
[[ -f ./ws.repos ]] && ./scripts/sync.sh

# build everything in ws/
./scripts/build.sh

# load the workspace environment into your current shell
source ws/install/setup.bash

# (now you can run/launch nodes, e.g.)
# ros2 run <package> <executable>
# ros2 launch <package> <launch_file.py>
\`\`\`

> Single-repo projects: \`./scripts/sync.sh\` isn’t needed (no \`ws.repos\`).  
> Tip: The build script forwards any extra args to \`colcon build\`. For example:  
> \`./scripts/build.sh --packages-select my_pkg --symlink-install\`

---

## Repository Layout

- \`ws/\` — colcon workspace
  - \`src/\` — packages ($src_note)
  - \`build/\`, \`install/\`, \`log/\` — generated; ignored by git
- \`.env\` — per-project env overrides (e.g., \`ROS_DOMAIN_ID\`, RMW choice)
- \`scripts/\` — helper scripts for syncing, freezing, building, cleaning, and creating packages
- (meta only) \`ws.repos\` — list of external repos to import into \`ws/src\`
- (meta only) \`ws.repos.lock\` — pinned exact commits for reproducibility

---

## Scripts Overview

### scripts/sync.sh
**Use when:** \(meta projects\) You want to materialize or update \`ws/src\` from \`ws.repos\`.

**What it does:**
- Creates \`ws/src\` if needed.
- \`vcs validate\` the \`ws.repos\` file.
- \`vcs import\` to clone any missing repos into \`ws/src\`.
- \`vcs pull\` to update existing repos to their remote heads.

**Notes:**
- Idempotent — run anytime to bring sources up to date.
- Respects whatever ref/branch \`ws.repos\` lists. If you want exact pins, see \`freeze.sh\` below.

**Run:**
\`\`\`bash
./scripts/sync.sh
\`\`\`

---

### scripts/freeze.sh
**Use when:** \(meta projects\) You want to **pin** the current state of \`ws/src\` for reproducible builds.

**What it does:**
- Exports the **exact commits** of all repos in \`ws/src\` to \`ws.repos.lock\` via:
  \`vcs export --exact ws/src > ws.repos.lock\`.

**Workflow:**
1. Run \`./scripts/sync.sh\` to fetch/update sources.
2. Verify everything builds.
3. Run \`./scripts/freeze.sh\` and **commit** \`ws.repos.lock\`.

**Run:**
\`\`\`bash
./scripts/freeze.sh
\`\`\`

---

### scripts/build.sh
**Use when:** You want to build the workspace.

**What it does:**
- Loads \`.env\` (e.g., \`ROS_DOMAIN_ID\`, \`RMW_IMPLEMENTATION\`).
- Sources \`/opt/ros/\${ROS_DISTRO:-kilted}/setup.bash\` (container-provided).
- (Optional) You can add an underlay source line in the script if you have one.
- Runs \`colcon build --symlink-install\` in \`ws/\`.

**Pass-through args:** Any extra args are forwarded to \`colcon build\`. Examples:
- Select packages:
  \`\`\`bash
  ./scripts/build.sh --packages-select my_pkg another_pkg
  \`\`\`
- CMake config:
  \`\`\`bash
  ./scripts/build.sh --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  \`\`\`

**Run:**
\`\`\`bash
./scripts/build.sh
\`\`\`

---

### scripts/clean.sh
**Use when:** You want to nuke build artifacts and logs.

**What it does:** \`rm -rf ws/build ws/install ws/log\`

**Run:**
\`\`\`bash
./scripts/clean.sh
\`\`\`

---

### scripts/create_pkg.sh
**Use when:** \(single projects\) You want to scaffold a new package under $(ws/src).

**What it does:**
- Ensures ROS env is sourced.
- Invokes $(ros2 pkg create) with the given name and dependencies, creating the package under $(ws/src).
- **Language selection:** Defaults to **C++** ($(ament_cmake)). Pass **$(--py) as the first argument** to create a **Python** package ($(ament_python)).

**Examples:**
\`\`\`bash
# Minimal (C++, default)
./scripts/create_pkg.sh my_pkg

# Minimal (Python)
./scripts/create_pkg.sh --py my_pkg

# With dependencies (C++)
./scripts/create_pkg.sh my_pkg --dependencies rclcpp std_msgs sensor_msgs

# With dependencies (Python)
./scripts/create_pkg.sh --py my_pkg --dependencies rclpy std_msgs sensor_msgs
\`\`\`

> Note: Run this **inside the container**.

---

## Lockfiles & Sources

- **\`ws.repos\`** (meta): Human-maintained list of repos + branches/versions to import into \`ws/src\`.
- **\`ws.repos.lock\`** (meta): Generated by \`freeze.sh\`, pins **exact commits** for reproducibility.
  - Commit this file to share a known-good state.
  - To re-sync to the locked commits, you can:
    \`\`\`bash
    vcs import ws/src < ws.repos.lock
    vcs pull ws/src
    \`\`\`

---

## Common Tasks

- **Switch RMW or Domain ID**: Edit \`.env\` (e.g., \`RMW_IMPLEMENTATION=rmw_fastrtps_cpp\`, \`ROS_DOMAIN_ID=23\`).
- **Rebuild after dependency changes**: Run \`./scripts/build.sh\` (optionally with \`--cmake-args\`).
- **Run tests**:
  \`\`\`bash
  cd ws
  colcon test && colcon test-result --verbose
  \`\`\`
- **Run a node** (after sourcing install in your shell session):
  \`\`\`bash
  source ws/install/setup.bash
  ros2 run <package> <executable>
  \`\`\`

---

## Troubleshooting

- **Colcon not found / ROS not sourced:** Use the container via the infra repo and make sure you entered the shell through \`workon.sh\`.
- **Meta repo not pulling updates:** Ensure \`ws.repos\` lists the branch you expect; run \`./scripts/sync.sh\` again.
- **Networking between projects:** Set a unique \`ROS_DOMAIN_ID\` in \`.env\` to avoid cross-talk with other running workspaces.

---

**Infra link:** This project expects the Docker/ROS environment from **ros-workspace**: https://github.com/Kyaw-Thiha/ros-workspace

EOF

# scripts/sync.sh
cat >"$proj_dir/scripts/sync.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$root/ws.repos" ]]; then
  mkdir -p "$root/ws/src"
  vcs validate < "$root/ws.repos"
  vcs import "$root/ws/src" < "$root/ws.repos"
  vcs pull "$root/ws/src"
else
  echo "No ws.repos found (single-repo or packages already present)."
fi
EOF
chmod +x "$proj_dir/scripts/sync.sh"

# scripts/freeze.sh (meta only)
cat >"$proj_dir/scripts/freeze.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -d "$root/ws/src" ]]; then
  echo "Nothing in ws/src to freeze. Run scripts/sync.sh first."
  exit 1
fi
vcs export --exact "$root/ws/src" > "$root/ws.repos.lock"
echo "Wrote $root/ws.repos.lock"
EOF
chmod +x "$proj_dir/scripts/freeze.sh"

# scripts/build.sh
cat >"$proj_dir/scripts/build.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# load env
set -a
[[ -f "$root/.env" ]] && source "$root/.env"
set +a

# source ROS and optional underlay (guard against nounset + unset AMENT_* vars)
set +u
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=$(command -v python3 || true)}"
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
# Example underlay:
# if [[ -f /work/underlay_ws/install/setup.bash ]]; then source /work/underlay_ws/install/setup.bash; fi
set -u

cd "$root/ws"
colcon build --symlink-install "$@"
EOF
chmod +x "$proj_dir/scripts/build.sh"

# scripts/clean.sh
cat >"$proj_dir/scripts/clean.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rm -rf "$root/ws/build" "$root/ws/install" "$root/ws/log"
echo "Cleaned build/install/log"
EOF
chmod +x "$proj_dir/scripts/clean.sh"

# scripts/create_pkg.sh (run INSIDE container for single-repo)
cat >"$proj_dir/scripts/create_pkg.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
usage(){ echo "Usage: scripts/create_pkg.sh [--py] <package_name> [--dependencies ...]"; }

build_type="ament_cmake"
[[ $# -ge 1 ]] || { usage; exit 1; }

if [[ ${1:-} == "--py" ]]; then
  build_type="ament_python"
  shift
fi

# ... keep everything above unchanged ...

pkg="$1"; shift || true
deps=("$@")
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$root/ws/src"
cd "$root/ws"

# Guard against nounset + unset AMENT_* vars when sourcing ROS env
set +u
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=$(command -v python3 || true)}"
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
set -u

# Build proper dependencies arg (accepts with or without a leading --dependencies)
dep_args=()
if [[ ${#deps[@]} -gt 0 ]]; then
  if [[ "${deps[0]}" == "--dependencies" ]]; then
    deps=("${deps[@]:1}")
  fi
  if [[ ${#deps[@]} -gt 0 ]]; then
    dep_args=(--dependencies "${deps[@]}")
  fi
fi

ros2 pkg create --build-type "$build_type" "${dep_args[@]}" --destination-directory src "$pkg"
echo "Created $build_type package at ws/src/$pkg"
EOF
chmod +x "$proj_dir/scripts/create_pkg.sh"

# -------- git init & first commit --------
if ((do_git)); then
  (
    cd "$proj_dir"
    git init -q
    git add .
    git commit -m "chore: scaffold ${ptype} ROS2 project template" >/dev/null
  )
fi

# -------- optional GitHub create/push --------
if [[ -n "$gh_owner" && $do_git -eq 1 ]]; then
  if command -v gh >/dev/null 2>&1; then
    echo "Creating GitHub repo: ${gh_owner}/${name} ($visibility)"
    visibility_flag="--${visibility}"
    gh repo create "${gh_owner}/${name}" $visibility_flag --source "$proj_dir" --remote origin ${do_push:+--push} || {
      echo "WARN: gh repo create failed; you can add a remote later:"
      echo "  cd \"$proj_dir\" && git remote add origin git@github.com:${gh_owner}/${name}.git && git push -u origin main"
    }
  else
    echo "NOTE: GitHub CLI 'gh' not found. To create remote later:"
    echo "  cd \"$proj_dir\""
    echo "  git remote add origin git@github.com:${gh_owner}/${name}.git"
    ((do_push)) && echo "  git push -u origin main"
  fi
fi

echo
echo "✅ Project created: $proj_dir"
echo "Next steps:"
echo "  1) Mount into your container:"
echo "       ./scripts/workon.sh \"$proj_dir\""
if [[ "$ptype" == "meta" ]]; then
  echo "  2) Inside container: ./scripts/sync.sh   # import repos into ws/src"
  echo "  3) Inside container: ./scripts/build.sh"
  echo "  4) (optional)       : ./scripts/freeze.sh"
else
  echo "  2) Inside container (optional): ./scripts/create_pkg.sh my_pkg --dependencies rclcpp std_msgs"
  echo "  3) Inside container: ./scripts/build.sh"
fi
