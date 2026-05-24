#!/usr/bin/env bash
# Set up a tuning worker on a fresh machine. Idempotent — re-run to update.
#
# Usage:
#   ./deploy_worker.sh [--root /shared/studies] [--project capra_v1] [--stage stage_a_icp_core]
#
# Assumes:
#   - Ubuntu 22.04 + ROS 2 Humble already installed
#   - Repo cloned, with worktree-sim-webots branch checked out at $WORKSPACE
#   - $WORKSPACE/install/setup.zsh exists (i.e. the workspace has been built once)

set -euo pipefail

ROOT="${ROOT:-$HOME/shared/studies}"
PROJECT="${PROJECT:-default_project}"
STAGE="${STAGE:-stage_a_icp_core}"
SEED="${SEED:-$RANDOM}"
DOMAIN="${DOMAIN:-122}"
MAX_TRIALS="${MAX_TRIALS:-10}"
WORKSPACE="${WORKSPACE:-$PWD}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --root) ROOT="$2"; shift 2 ;;
    --project) PROJECT="$2"; shift 2 ;;
    --stage) STAGE="$2"; shift 2 ;;
    --seed) SEED="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --max-trials) MAX_TRIALS="$2"; shift 2 ;;
    --workspace) WORKSPACE="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 1 ;;
  esac
done

if ! command -v webots >/dev/null 2>&1; then
  echo "Webots not found — installing (this requires sudo)..."
  sudo mkdir -p /etc/apt/keyrings
  curl -fsSL https://cyberbotics.com/Cyberbotics.asc \
    | sudo gpg --dearmor -o /etc/apt/keyrings/cyberbotics.gpg
  echo 'deb [arch=amd64 signed-by=/etc/apt/keyrings/cyberbotics.gpg] https://cyberbotics.com/debian binary-amd64/' \
    | sudo tee /etc/apt/sources.list.d/cyberbotics.list
  sudo apt-get update -y
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    webots ros-humble-webots-ros2 ros-humble-rtabmap-launch xvfb
fi

pip install --user --upgrade 'optuna>=3.4' optuna-dashboard cma pyyaml numpy

cd "$WORKSPACE"
if [[ ! -d install/rove_tuning_orchestrator ]]; then
  source /opt/ros/humble/setup.bash
  colcon build --packages-select rove_sim_webots rove_tuning_orchestrator --symlink-install
fi

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

mkdir -p "$ROOT/$PROJECT/phase1_sim"

echo "==========================="
echo " worker: $(hostname)"
echo "  root:        $ROOT"
echo "  project:     $PROJECT"
echo "  stage:       $STAGE"
echo "  seed:        $SEED"
echo "  domain:      $DOMAIN"
echo "  max trials:  $MAX_TRIALS"
echo "==========================="

exec python3 -m rove_tuning_orchestrator.worker \
  --root "$ROOT" \
  --project "$PROJECT" \
  --stage "$STAGE" \
  --seed "$SEED" \
  --sim-domain-id "$DOMAIN" \
  --max-trials "$MAX_TRIALS"
