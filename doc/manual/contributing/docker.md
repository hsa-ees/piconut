# Docker

**Author: Niklas Sirch 2025**

This document explains how to build the development Docker image, start a container, and optionally customize the container on startup.

## Build the Docker image

The image is created with the helper script so user IDs inside the image match your host user
(required for GUI tooling to work correctly). From the repository root run:

```console
$ cd tools/docker
$ ./docker-create-piconut-image.sh --cloud-raw --qt
```

If you prefer to build a fully customized image, run the same script with other options. The build
is resource intensive; we recommend having at least ~24 GB RAM and ~80 GB free disk space.

For full options and help:

```console
$ ./docker-create-piconut-image.sh --help
```

Tip: use the `--cloud-raw` option to start from the provided cloud image if you don't want to
rebuild everything locally.

## Run the Docker container

Start the container with Docker Compose and open a shell inside it:

```console
$ docker compose up -d
$ docker exec -it piconut_container bash
```

If your environment uses `docker compose exec` instead of `docker exec`, you can use:

```console
$ docker compose exec piconut_container bash
```

## Devcontainer

The Docker image can be used as a VS Code Dev Container. See the official docs for full details:
https://code.visualstudio.com/docs/devcontainers/containers

In Piconut it should be straightforward as in install the devcontainer extension and click on
`Reopen in devcontainer`.

## Customize the container on startup

Images are immutable, so to persist customizations you can provide a script that runs when the
container is created. Create a file named `compose-entrypoint.local.sh` and make it executable;
place it alongside your Docker Compose files or in `tools/docker` and mount it in the container
according to your compose setup.

Example `compose-entrypoint.local.sh` that appends convenience settings to `~/.bashrc`:

```bash
#!/bin/bash
set -e

if [ -f $HOME/local-entrypoint.ran ]; then
  exit 0
fi

cat <<'EOS' >> ~/.bashrc
# Set build directory inside the source tree
export PN_BUILD_DIR="$HOME/piconut/build"

# Use vi keybindings
set -o vi

# code binary is installed after container is created so we install extensions with on bash startup later
if command -v code >/dev/null 2>&1 && [ ! -f "$HOME/.vscode-extensions-installed" ]; then
  code --install-extension streetsidesoftware.code-spell-checker
  code --install-extension arturock.gitstash
  touch "$HOME/.vscode-extensions-installed"
fi
EOS

touch $HOME/local-entrypoint.ran
```

## Install additional packages

For testing, you may want to install additional packages inside the container. The container image
does not include `sudo`, so to install packages you can either modify the Dockerfile or use
`docker exec` to run `apt-get` as root.

Example:

```console
$ docker exec -it --user root piconut_container bash
root@container-id:/home/user# apt-get update
root@container-id:/home/user# apt-get install -y <package-name>
```
