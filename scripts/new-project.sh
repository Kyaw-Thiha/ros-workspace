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

# README
cat >"$proj_dir/README.md" <<EOF
# $name

Type: **$ptype**

## Quick start

On the host (outside the container), from your infra repo:

\`\`\`bash
# mount this project into your ROS container
/path/to/ros-workspace/scripts/workon.sh "$proj_dir"
\`\`\`

Once inside the container:

\`\`\`bash
# (meta) fetch sources
[[ -f ./ws.repos ]] && ./scripts/sync.sh

# build
./scripts/build.sh
\`\`\`

## Layout

- \`ws/\` — colcon workspace
  - \`src/\` — packages (${ptype== "meta" && "populated by vcstool" || "committed in this repo"})
  - \`build/ install/ log/\` — ignored
- \`.env\` — per-project env (e.g., ROS_DOMAIN_ID)
- \`scripts/\` — helper scripts (sync/freeze/build/clean, create_pkg)
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

# source ROS and optional underlay
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
# Example underlay:
# [ -f /work/underlay_ws/install/setup.bash ] && source /work/underlay_ws/install/setup.bash

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
if [[ $# -lt 1 ]]; then
  echo "Usage: scripts/create_pkg.sh <package_name> [--dependencies rclcpp std_msgs ...]"
  exit 1
fi
pkg="$1"; shift || true
deps=("$@")
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$root/ws/src"
cd "$root/ws"
# Ensure ROS env
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
ros2 pkg create "$pkg" "${deps[@]/#/--dependencies }" --build-type ament_cmake --destination-directory src
echo "Created package at ws/src/$pkg"
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
