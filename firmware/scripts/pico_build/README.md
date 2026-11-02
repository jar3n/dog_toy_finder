# Pico firmware build

Bash pipeline for compiling the firmware in
[`firmware/pico_microros/`](../../pico_microros/).

The script resolves the Pico SDK, ARM GCC toolchain, CMake, and Ninja, configures
the CMake project, builds it, and stages the flashable output.

## Quick start

From anywhere in the repository:

```bash
firmware/scripts/pico_build/build.sh
```

Clean release build:

```bash
firmware/scripts/pico_build/build.sh --clean --build-type Release
```

Run `firmware/scripts/pico_build/build.sh --help` for all options.

## Requirements

- **Pico SDK** - either the version installed by the Raspberry Pi Pico VS Code
  extension under `~/.pico-sdk/sdk/`, or a directory in `PICO_SDK_PATH`.
- **ARM GCC toolchain** - `arm-none-eabi-gcc`, either from the VS Code extension
  under `~/.pico-sdk/toolchain/`, or from `PICO_TOOLCHAIN_PATH` / `PATH`.
- **CMake** and **Ninja** - from `PATH` or the VS Code extension copies under
  `~/.pico-sdk/`.
- **micro-ROS library** - `firmware/pico_microros/lib/libmicroros/libmicroros.a`.
  Build it with the `microros_builder` container if it is missing:

  ```bash
  (cd docker && docker compose run --rm microros_builder bash build-library.sh)
  ```

  Then copy `docker/microros_builder/microros/artifacts/` into
  `firmware/pico_microros/lib/libmicroros/`. Use `--no-lib-check` only when you
  intentionally want to configure without the library.

## Output

By default the build directory is `firmware/pico_microros/build/`. After a
successful build, flashable files are copied to:

```text
firmware/pico_microros/build/artifacts/
├── dtf_pico_fw.uf2
├── dtf_pico_fw.elf
├── dtf_pico_fw.bin
├── dtf_pico_fw.hex
├── dtf_pico_fw.elf.map
└── manifest.txt
```

`manifest.txt` records the board, build type, SDK/toolchain paths, git commit,
and artifact sizes. `compile_commands.json` is generated in the build directory
for editors and clangd.

To flash, hold **BOOTSEL**, connect the Pico over USB, and copy the `.uf2` file
to the mounted `RPI-RP2` drive.

## Options

| Option | Description |
|---|---|
| `-b, --build-dir DIR` | Build directory (default `firmware/pico_microros/build`) |
| `-B, --board BOARD` | Pico board name (default `PICO_BOARD` or `pico2_w`) |
| `-t, --build-type TYPE` | `Debug`, `Release`, `RelWithDebInfo`, `MinSizeRel` |
| `-j, --jobs N` | Parallel jobs (default: CPU count) |
| `-T, --target NAME` | CMake target (default `all`) |
| `-c, --clean` | Remove the build directory before configuring |
| `-v, --verbose` | Show every compiler command |
| `-n, --dry-run` | Print commands without executing them |
| `--configure-only` | Configure CMake, skip compilation |
| `--no-stage` | Leave artifacts in the build root |
| `--no-lib-check` | Skip the `libmicroros.a` existence check |
| `--sdk PATH` | Pico SDK directory |
| `--toolchain PATH` | ARM GCC toolchain root |
| `--cmake PATH` | CMake executable |
| `--ninja PATH` | Ninja executable |
| `-- <args>` | Extra arguments passed to CMake configure |

Environment variables `PICO_SDK_PATH`, `PICO_TOOLCHAIN_PATH`, `PICO_BOARD`, and
`CMAKE_BUILD_TYPE` provide the same defaults.

## Examples

Build into a separate directory without touching the normal build:

```bash
firmware/scripts/pico_build/build.sh --build-dir /tmp/dtf-pico-build
```

Build a different board in release mode:

```bash
firmware/scripts/pico_build/build.sh --board pico_w --build-type Release
```

CI-style clean build:

```bash
firmware/scripts/pico_build/build.sh --clean --build-type Release --jobs 4
```

## Building inside the microros container

The builder image already contains CMake, Ninja, the ARM toolchain, and the Pico
SDK at `/opt/pico-sdk`. Use a nested build directory so the container does not
reuse host CMake cache paths:

```bash
(cd docker && docker compose run --rm -w /workspace microros_builder \
  bash firmware/scripts/pico_build/build.sh --clean \
  --build-dir firmware/pico_microros/build/docker)
```

## Troubleshooting

- **CMake generator mismatch** - switching between Ninja and Make, or moving a
  build directory between host and container, requires `--clean`.
- **SDK/toolchain warning** - `CMakeLists.txt` contains a Pico VS Code block that
  pins SDK/toolchain versions when `~/.pico-sdk/cmake/pico-vscode.cmake` exists.
  The script reports the paths CMake actually used.
- **Missing `lwipopts.h`** - `config/` is on the include path in
  `CMakeLists.txt`; do not remove it when reorganizing the transport code.
