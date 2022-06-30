#!/usr/bin/env zsh

# Get the current directory.
script_source=$(readlink -f "$0")
script_dir=$(dirname "$script_source")

docker_ctx=${script_dir}/docker_ctx
echo "Creating build context in: ${docker_ctx}"

# Create the context we use for setting up the images.
mkdir -p "${docker_ctx}"
"$script_dir/copy_to_ctx.sh"

# Build the docker image.
docker-compose -f "${script_dir}/docker-compose.yml" build

# We don't remove the context here as docker-compose requires it to be present.