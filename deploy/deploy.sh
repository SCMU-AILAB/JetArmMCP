#!/usr/bin/env bash
# 把源码同步到 Jetson 并在那边编译。用法：deploy/deploy.sh [--install-service]
set -euo pipefail
HOST="${JETSON:-jetson@192.168.0.11}"
cd "$(dirname "$0")/.."

rsync -az --delete --exclude target/ --exclude reference/ --exclude .git/ ./ "$HOST:JetArmMCP/"

ssh "$HOST" bash -s <<'REMOTE'
set -euo pipefail
if ! command -v cargo >/dev/null && [ ! -x ~/.cargo/bin/cargo ]; then
  echo "安装 Rust 工具链…"
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
fi
. ~/.cargo/env
# v4l 的绑定在编译时需要 libclang
if ! ldconfig -p | grep -q libclang; then
  echo "缺少 libclang：请在 Jetson 上运行 sudo apt install -y libclang-dev 后重试" >&2
  exit 1
fi
cd ~/JetArmMCP
cargo build --release
REMOTE

if [ "${1:-}" = "--install-service" ]; then
  ssh -t "$HOST" "sudo cp ~/JetArmMCP/deploy/jetarm-mcp.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now jetarm-mcp && systemctl status --no-pager jetarm-mcp"
else
  ssh "$HOST" "systemctl is-active --quiet jetarm-mcp 2>/dev/null && sudo -n systemctl restart jetarm-mcp || true"
fi
echo "完成。Mac 端接入：claude mcp add --transport http jetarm http://${HOST#*@}:8848/mcp"
