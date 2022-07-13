#!/usr/bin/env zsh

# Get the current directory.
script_source=$(readlink -f "$0")
script_dir=$(dirname "$script_source")
docker_ctx=${script_dir}/docker_ctx
ws=${docker_ctx}/ws/src/maplab/
ws_src=${ws}/..
project_dir=${script_dir}/..

# Copy the project files to the context.
mkdir -p "${ws}"
rsync -a \
  --exclude '*.git' \
  --exclude 'deploy/' \
  "${project_dir}" "${ws}"

# Copy the installation script to the context.
cp "${script_dir}/install_base.sh" "${docker_ctx}"
cp "${script_dir}/install_ros.sh" "${docker_ctx}"
cp "${script_dir}/install_packages.sh" "${docker_ctx}"
cp "${script_dir}/set_env.sh" "${docker_ctx}"
cp "${script_dir}/compile.sh" "${docker_ctx}"
