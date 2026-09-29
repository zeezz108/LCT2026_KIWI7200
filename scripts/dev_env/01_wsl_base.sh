#!/usr/bin/env bash
# Base setup of the Ubuntu 22.04 WSL distro (run as root).
set -euo pipefail

DEV_USER=dev

if ! id "$DEV_USER" &>/dev/null; then
  useradd -m -s /bin/bash -G sudo "$DEV_USER"
fi
echo "$DEV_USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$DEV_USER
chmod 440 /etc/sudoers.d/$DEV_USER

# systemd is needed for dockerd; Windows PATH (with spaces) confuses cmake/colcon
cat > /etc/wsl.conf <<EOF
[boot]
systemd=true

[user]
default=$DEV_USER

[interop]
appendWindowsPath=false
EOF

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get upgrade -y
apt-get install -y --no-install-recommends \
  locales ca-certificates curl wget gnupg lsb-release software-properties-common \
  build-essential cmake git rsync htop pv unzip file

locale-gen en_US.UTF-8
update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

echo "01_wsl_base: OK"
