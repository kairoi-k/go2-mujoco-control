#!/usr/bin/env bash
# Bootstrap the Unitree Go2 MuJoCo research environment on Ubuntu/WSL2.
# Installs system dependencies, MuJoCo, Unitree SDK2, and builds the simulator
# plus C++ examples. Review before running because sudo is required.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mujoco_ver="${MUJOCO_VERSION:-3.3.6}"
mujoco_dir="${HOME}/.mujoco/mujoco-${mujoco_ver}"
mujoco_src_dir="${HOME}/.cache/mujoco-source-${mujoco_ver}"
sdk2_src="${HOME}/.cache/unitree_sdk2"
sdk2_prefix="${UNITREE_SDK2_PREFIX:-/opt/unitree_robotics}"

echo "[setup] repo=${repo_root}"

sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
  build-essential \
  g++ \
  libstdc++-14-dev \
  cmake \
  git \
  patch \
  wget \
  curl \
  ca-certificates \
  libyaml-cpp-dev \
  libspdlog-dev \
  libfmt-dev \
  libboost-all-dev \
  libglfw3-dev \
  libeigen3-dev \
  libgl1-mesa-dev \
  mesa-utils \
  xvfb \
  python3-pip \
  python3-venv

export CC="${CC:-gcc}"
export CXX="${CXX:-g++}"
echo 'int main(){return 0;}' > /tmp/go2_env_cxx_smoke.cpp
"${CXX}" /tmp/go2_env_cxx_smoke.cpp -o /tmp/go2_env_cxx_smoke
rm -f /tmp/go2_env_cxx_smoke /tmp/go2_env_cxx_smoke.cpp

mkdir -p "${HOME}/.mujoco" "${HOME}/downloads" "${HOME}/.cache"
if [[ ! -d "${mujoco_dir}" ]]; then
  archive="${HOME}/downloads/mujoco-${mujoco_ver}-linux-x86_64.tar.gz"
  wget -q \
    "https://github.com/google-deepmind/mujoco/releases/download/${mujoco_ver}/mujoco-${mujoco_ver}-linux-x86_64.tar.gz" \
    -O "${archive}"
  tar -xzf "${archive}" -C "${HOME}/.mujoco"
fi

# Release binaries provide include/ and lib/ but not the viewer sources that
# this repository compiles directly. Hydrate exactly the matching tag's
# simulate/ tree into the binary distribution before applying our pinned patch.
if [[ ! -f "${mujoco_dir}/simulate/simulate.cc" ]]; then
  src_archive="${HOME}/downloads/mujoco-${mujoco_ver}-source.tar.gz"
  rm -rf "${mujoco_src_dir}"
  mkdir -p "${mujoco_src_dir}"
  wget -q \
    "https://github.com/google-deepmind/mujoco/archive/refs/tags/${mujoco_ver}.tar.gz" \
    -O "${src_archive}"
  tar -xzf "${src_archive}" -C "${mujoco_src_dir}" --strip-components=1
  test -f "${mujoco_src_dir}/simulate/simulate.cc"
  rm -rf "${mujoco_dir}/simulate"
  cp -a "${mujoco_src_dir}/simulate" "${mujoco_dir}/simulate"
fi

ln -sfn "${mujoco_dir}" "${repo_root}/simulate/mujoco"
test -f "${repo_root}/simulate/mujoco/include/mujoco/mujoco.h"
test -f "${repo_root}/simulate/mujoco/simulate/simulate.cc"
# The simulator source calls the repository's passive render-snapshot seam.
# A fresh upstream source tree does not contain it, so bootstrap must apply
# the pinned, idempotent repository patch before compiling simulate/.
bash "${repo_root}/patches/apply_mujoco_passive_render_patch.sh"

if [[ ! -f "${sdk2_prefix}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" ]]; then
  if [[ ! -d "${sdk2_src}/.git" ]]; then
    rm -rf "${sdk2_src}"
    git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git "${sdk2_src}"
  fi
  cmake -S "${sdk2_src}" -B "${sdk2_src}/build" -DCMAKE_INSTALL_PREFIX="${sdk2_prefix}"
  cmake --build "${sdk2_src}/build" -j"$(nproc)"
  sudo cmake --install "${sdk2_src}/build"
fi

python3 -m pip install --user --break-system-packages -q matplotlib pandas numpy pyyaml

cmake -S "${repo_root}/simulate" -B "${repo_root}/simulate/build"
cmake --build "${repo_root}/simulate/build" -j"$(nproc)"
cmake -S "${repo_root}/example/cpp" -B "${repo_root}/example/cpp/build"
cmake --build "${repo_root}/example/cpp/build" -j"$(nproc)"

"${repo_root}/example/cpp/build/test_go2_forward_kinematics" >/dev/null
"${repo_root}/example/cpp/build/test_go2_inverse_kinematics" >/dev/null

echo "[setup] ready: simulator and examples built; FK/IK smoke checks passed"
