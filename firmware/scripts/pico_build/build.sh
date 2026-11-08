#!/usr/bin/env bash
#
# Build the Pico micro-ROS firmware.
#
# The script locates the Pico SDK, ARM toolchain, CMake, and Ninja (preferring
# the copies installed by the Raspberry Pi Pico VS Code extension), configures
# CMake, builds the firmware, and stages the flashable artifacts.
#
# Run with --help for usage.

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
FIRMWARE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
REPO_ROOT="$(cd -- "${FIRMWARE_DIR}/.." && pwd -P)"
SOURCE_DIR="${FIRMWARE_DIR}/pico_microros"

if [[ -t 2 && -z "${NO_COLOR:-}" ]]; then
  COLOR_INFO=$'\033[1;34m'
  COLOR_OK=$'\033[1;32m'
  COLOR_WARN=$'\033[1;33m'
  COLOR_ERROR=$'\033[1;31m'
  COLOR_RESET=$'\033[0m'
else
  COLOR_INFO=""
  COLOR_OK=""
  COLOR_WARN=""
  COLOR_ERROR=""
  COLOR_RESET=""
fi

log() {
  printf '%s==>%s %s\n' "${COLOR_INFO}" "${COLOR_RESET}" "$*"
}

success() {
  printf '%s==>%s %s\n' "${COLOR_OK}" "${COLOR_RESET}" "$*"
}

warn() {
  printf '%s==> warning:%s %s\n' "${COLOR_WARN}" "${COLOR_RESET}" "$*" >&2
}

die() {
  printf '%s==> error:%s %s\n' "${COLOR_ERROR}" "${COLOR_RESET}" "$*" >&2
  exit 1
}

on_error() {
  local status=$?
  printf '%s==> error:%s unexpected command failure (exit %s)\n' \
    "${COLOR_ERROR}" "${COLOR_RESET}" "${status}" >&2
  exit "${status}"
}
trap on_error ERR

usage() {
  cat <<'EOF'
Build the Pico micro-ROS firmware.

Usage:
  build.sh [options]

Options:
  -b, --build-dir DIR    Build directory (default: firmware/pico_microros/build)
  -B, --board BOARD      Pico board (default: $PICO_BOARD or pico2_w)
  -t, --build-type TYPE  Debug, Release, RelWithDebInfo, or MinSizeRel
                         (default: $CMAKE_BUILD_TYPE or Debug)
  -j, --jobs N           Parallel build jobs (default: number of CPUs)
  -T, --target NAME      CMake target to build (default: all)
  -c, --clean            Remove the build directory before configuring
  -v, --verbose          Show every compiler command
  -n, --dry-run          Print commands without running them
      --configure-only   Configure CMake but do not compile
      --no-stage         Leave artifacts in the build root (no artifacts/ copy)
      --no-lib-check     Do not require lib/libmicroros/libmicroros.a
      --sdk PATH         Pico SDK directory (overrides $PICO_SDK_PATH)
      --toolchain PATH   ARM GCC toolchain root (overrides $PICO_TOOLCHAIN_PATH)
      --cmake PATH       cmake executable to use
      --ninja PATH       ninja executable to use
  -h, --help             Show this help

Environment:
  PICO_SDK_PATH       Pico SDK directory
  PICO_TOOLCHAIN_PATH ARM GCC toolchain root
  PICO_BOARD          Pico board name
  CMAKE_BUILD_TYPE    Default CMake build type
  NO_COLOR            Disable colored output

Examples:
  build.sh
  build.sh --clean --build-type Release
  build.sh --build-dir /tmp/dtf-build --jobs 8
EOF
}

run() {
  if [[ ${DRY_RUN} -eq 1 ]]; then
    printf '    +'
    printf ' %q' "$@"
    printf '\n'
  else
    "$@"
  fi
}

run_or_die() {
  local failure_message=$1
  shift
  if [[ ${DRY_RUN} -eq 1 ]]; then
    run "$@"
    return 0
  fi
  if ! "$@"; then
    die "${failure_message}"
  fi
}

abspath() {
  if command -v realpath >/dev/null 2>&1; then
    realpath -m -- "$1"
  elif [[ "$1" = /* ]]; then
    printf '%s\n' "$1"
  else
    printf '%s/%s\n' "${PWD}" "$1"
  fi
}

if sort -V </dev/null >/dev/null 2>&1; then
  SORT_VERSION=(sort -V)
else
  SORT_VERSION=(sort)
fi

latest_subdir() {
  local base=$1
  [[ -d "${base}" ]] || return 1
  find "${base}" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' \
    2>/dev/null | "${SORT_VERSION[@]}" | tail -n 1
}

latest_arm_toolchain() {
  local base=$1
  local dir
  [[ -d "${base}" ]] || return 1
  while IFS= read -r dir; do
    if [[ -x "${dir}/bin/arm-none-eabi-gcc" ]]; then
      printf '%s\n' "${dir}"
      return 0
    fi
  done < <(find "${base}" -mindepth 1 -maxdepth 1 -type d -print \
    2>/dev/null | "${SORT_VERSION[@]}" -r)
  return 1
}

latest_tool_dir() {
  local base=$1
  local subdir=$2
  local config=$3
  local version
  local candidate
  [[ -d "${base}" ]] || return 1
  while IFS= read -r version; do
    candidate="${base}/${version}/${subdir}"
    if [[ -f "${candidate}/${config}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done < <(find "${base}" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' \
    2>/dev/null | "${SORT_VERSION[@]}" -r)
  return 1
}

human_size() {
  local bytes=$1
  if command -v numfmt >/dev/null 2>&1; then
    numfmt --to=iec-i --suffix=B "${bytes}" 2>/dev/null || printf '%sB' "${bytes}"
  else
    printf '%sB' "${bytes}"
  fi
}

# Defaults ------------------------------------------------------------------
BUILD_DIR=""
BOARD="${PICO_BOARD:-pico2_w}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Debug}"
JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || nproc 2>/dev/null || echo 1)"
TARGET="all"
CLEAN=0
VERBOSE=0
DRY_RUN=0
CONFIGURE_ONLY=0
STAGE=1
CHECK_LIB=1
SDK_ARG=""
TOOLCHAIN_ARG=""
CMAKE_ARG=""
NINJA_ARG=""
EXTRA_CMAKE_ARGS=()

# Arguments -----------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    -b|--build-dir)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      BUILD_DIR=$2
      shift 2
      ;;
    -B|--board)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      BOARD=$2
      shift 2
      ;;
    -t|--build-type)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      BUILD_TYPE=$2
      shift 2
      ;;
    -j|--jobs)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      JOBS=$2
      shift 2
      ;;
    -T|--target)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      TARGET=$2
      shift 2
      ;;
    -c|--clean)
      CLEAN=1
      shift
      ;;
    -v|--verbose)
      VERBOSE=1
      shift
      ;;
    -n|--dry-run)
      DRY_RUN=1
      shift
      ;;
    --configure-only)
      CONFIGURE_ONLY=1
      shift
      ;;
    --no-stage)
      STAGE=0
      shift
      ;;
    --no-lib-check)
      CHECK_LIB=0
      shift
      ;;
    --sdk)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      SDK_ARG=$2
      shift 2
      ;;
    --toolchain)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      TOOLCHAIN_ARG=$2
      shift 2
      ;;
    --cmake)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      CMAKE_ARG=$2
      shift 2
      ;;
    --ninja)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      NINJA_ARG=$2
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_CMAKE_ARGS+=("$@")
      break
      ;;
    *)
      die "Unknown option: $1 (run with --help for usage)"
      ;;
  esac
done

# Validate arguments --------------------------------------------------------
case "${BUILD_TYPE,,}" in
  debug) BUILD_TYPE=Debug ;;
  release) BUILD_TYPE=Release ;;
  relwithdebinfo) BUILD_TYPE=RelWithDebInfo ;;
  minsizerel) BUILD_TYPE=MinSizeRel ;;
  *) die "Invalid build type '${BUILD_TYPE}' (use Debug, Release, RelWithDebInfo, or MinSizeRel)" ;;
esac

[[ "${JOBS}" =~ ^[1-9][0-9]*$ ]] || die "Jobs must be a positive integer (got '${JOBS}')"
[[ -n "${BOARD}" ]] || die "Board name must not be empty"
[[ -n "${TARGET}" ]] || die "Target must not be empty"
[[ -f "${SOURCE_DIR}/CMakeLists.txt" ]] || die "Firmware CMakeLists.txt not found in ${SOURCE_DIR}"

if [[ -z "${BUILD_DIR}" ]]; then
  BUILD_DIR="${SOURCE_DIR}/build"
fi
BUILD_DIR="$(abspath "${BUILD_DIR}")"

if [[ ${CLEAN} -eq 1 ]]; then
  case "${BUILD_DIR}" in
    /|"${REPO_ROOT}"|"${FIRMWARE_DIR}"|"${SOURCE_DIR}")
      die "Refusing to remove ${BUILD_DIR}"
      ;;
  esac
  if [[ ${#BUILD_DIR} -lt 4 ]]; then
    die "Refusing to remove suspicious build directory: ${BUILD_DIR}"
  fi
fi

# Discover toolchain and SDK ------------------------------------------------
SDK="${SDK_ARG:-${PICO_SDK_PATH:-}}"
if [[ -z "${SDK}" ]]; then
  version="$(latest_subdir "${HOME}/.pico-sdk/sdk" || true)"
  if [[ -n "${version}" ]]; then
    SDK="${HOME}/.pico-sdk/sdk/${version}"
  elif [[ -d /opt/pico-sdk ]]; then
    SDK=/opt/pico-sdk
  fi
fi
[[ -n "${SDK}" ]] || die "Pico SDK not found. Set PICO_SDK_PATH or install the Pico VS Code extension."
[[ -f "${SDK}/pico_sdk_init.cmake" ]] || die "Not a Pico SDK directory: ${SDK}"
SDK="$(cd -- "${SDK}" && pwd -P)"

TOOLCHAIN="${TOOLCHAIN_ARG:-${PICO_TOOLCHAIN_PATH:-}}"
if [[ -z "${TOOLCHAIN}" ]]; then
  TOOLCHAIN="$(latest_arm_toolchain "${HOME}/.pico-sdk/toolchain" || true)"
fi
if [[ -z "${TOOLCHAIN}" ]]; then
  gcc_path="$(command -v arm-none-eabi-gcc || true)"
  if [[ -n "${gcc_path}" ]]; then
    TOOLCHAIN="$(dirname -- "$(dirname -- "${gcc_path}")")"
  fi
fi
[[ -n "${TOOLCHAIN}" ]] || die "ARM GCC toolchain not found. Set PICO_TOOLCHAIN_PATH or install arm-none-eabi-gcc."
[[ -x "${TOOLCHAIN}/bin/arm-none-eabi-gcc" ]] || die "arm-none-eabi-gcc not found under ${TOOLCHAIN}/bin"
TOOLCHAIN="$(cd -- "${TOOLCHAIN}" && pwd -P)"

CMAKE_BIN="${CMAKE_ARG:-}"
if [[ -z "${CMAKE_BIN}" ]]; then
  CMAKE_BIN="$(command -v cmake || true)"
fi
if [[ -z "${CMAKE_BIN}" ]]; then
  version="$(latest_subdir "${HOME}/.pico-sdk/cmake" || true)"
  if [[ -n "${version}" ]]; then
    CMAKE_BIN="${HOME}/.pico-sdk/cmake/${version}/bin/cmake"
  fi
fi
[[ -n "${CMAKE_BIN}" ]] || die "cmake not found. Install CMake or the Pico VS Code extension."
[[ -x "${CMAKE_BIN}" ]] || die "cmake is not executable: ${CMAKE_BIN}"

NINJA_BIN="${NINJA_ARG:-}"
if [[ -z "${NINJA_BIN}" ]]; then
  NINJA_BIN="$(command -v ninja || true)"
fi
if [[ -z "${NINJA_BIN}" ]]; then
  version="$(latest_subdir "${HOME}/.pico-sdk/ninja" || true)"
  if [[ -n "${version}" ]]; then
    NINJA_BIN="${HOME}/.pico-sdk/ninja/${version}/ninja"
  fi
fi

if [[ -n "${NINJA_BIN}" && -x "${NINJA_BIN}" ]]; then
  GENERATOR="Ninja"
  MAKE_PROGRAM="${NINJA_BIN}"
else
  NINJA_BIN=""
  MAKE_PROGRAM="$(command -v make || true)"
  [[ -n "${MAKE_PROGRAM}" ]] || die "Neither ninja nor make found."
  GENERATOR="Unix Makefiles"
fi

PIOASM_DIR="$(latest_tool_dir "${HOME}/.pico-sdk/tools" "pioasm" "pioasmConfig.cmake" || true)"
PICOTOOL_DIR="$(latest_tool_dir "${HOME}/.pico-sdk/picotool" "picotool" "picotoolConfig.cmake" || true)"

MICROROS_LIB="${SOURCE_DIR}/lib/libmicroros/libmicroros.a"
if [[ ${CHECK_LIB} -eq 1 && ! -f "${MICROROS_LIB}" ]]; then
  die "micro-ROS library not found: ${MICROROS_LIB}
Build it with the microros_builder container first:
  (cd docker && docker compose run --rm microros_builder bash build-library.sh)"
fi

# Build ---------------------------------------------------------------------
log "Pico firmware build"
printf '    source:    %s\n' "${SOURCE_DIR}"
printf '    build:     %s\n' "${BUILD_DIR}"
printf '    board:     %s\n' "${BOARD}"
printf '    type:      %s\n' "${BUILD_TYPE}"
printf '    sdk:       %s\n' "${SDK}"
printf '    toolchain: %s\n' "${TOOLCHAIN}"
printf '    cmake:     %s\n' "${CMAKE_BIN}"
printf '    generator: %s\n' "${GENERATOR}"

if [[ ${CLEAN} -eq 1 ]]; then
  log "Removing ${BUILD_DIR}"
  if [[ ${DRY_RUN} -eq 0 && -d "${BUILD_DIR}" ]]; then
    rm -rf -- "${BUILD_DIR}"
  fi
fi

if [[ ${DRY_RUN} -eq 0 ]]; then
  mkdir -p -- "${BUILD_DIR}"
fi

configure_args=(
  -S "${SOURCE_DIR}"
  -B "${BUILD_DIR}"
  -G "${GENERATOR}"
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
  -DCMAKE_MAKE_PROGRAM="${MAKE_PROGRAM}"
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  -DPICO_BOARD="${BOARD}"
  -DPICO_SDK_PATH="${SDK}"
  -DPICO_TOOLCHAIN_PATH="${TOOLCHAIN}"
)
if [[ -n "${PIOASM_DIR}" ]]; then
  configure_args+=(-Dpioasm_DIR="${PIOASM_DIR}")
fi
if [[ -n "${PICOTOOL_DIR}" ]]; then
  configure_args+=(-Dpicotool_DIR="${PICOTOOL_DIR}")
fi
if [[ ${#EXTRA_CMAKE_ARGS[@]} -gt 0 ]]; then
  configure_args+=("${EXTRA_CMAKE_ARGS[@]}")
fi

log "Configuring"
run_or_die "CMake configure failed. If the SDK, toolchain, generator, or build type changed, rerun with --clean." \
  "${CMAKE_BIN}" "${configure_args[@]}"

if [[ ${DRY_RUN} -eq 0 && -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
  effective_sdk="$(sed -n 's/^PICO_SDK_PATH:PATH=//p' "${BUILD_DIR}/CMakeCache.txt" | tail -n 1)"
  effective_toolchain="$(sed -n 's/^PICO_TOOLCHAIN_PATH:[^=]*=//p' "${BUILD_DIR}/CMakeCache.txt" | tail -n 1)"
  if [[ -n "${effective_sdk}" && "${effective_sdk}" != "${SDK}" ]]; then
    warn "CMake resolved the SDK to ${effective_sdk} (configured: ${SDK})"
  fi
  if [[ -n "${effective_toolchain}" && "${effective_toolchain}" != "${TOOLCHAIN}" ]]; then
    warn "CMake resolved the toolchain to ${effective_toolchain} (configured: ${TOOLCHAIN})"
  fi
fi

if [[ ${CONFIGURE_ONLY} -eq 1 ]]; then
  success "Configure complete"
  exit 0
fi

build_args=(--build "${BUILD_DIR}" --parallel "${JOBS}" --target "${TARGET}")
if [[ ${VERBOSE} -eq 1 ]]; then
  build_args+=(--verbose)
fi

log "Building (${JOBS} jobs)"
run_or_die "Firmware build failed." "${CMAKE_BIN}" "${build_args[@]}"

# Stage artifacts -----------------------------------------------------------
artifacts=()
if [[ ${DRY_RUN} -eq 0 ]]; then
  while IFS= read -r file; do
    artifacts+=("${file}")
  done < <(find "${BUILD_DIR}" -maxdepth 1 -type f \
    \( -name '*.uf2' -o -name '*.elf' -o -name '*.bin' -o -name '*.hex' -o -name '*.map' \) \
    -print | sort)
fi

if [[ ${DRY_RUN} -eq 0 && ${#artifacts[@]} -eq 0 ]]; then
  if [[ "${TARGET}" = "all" ]]; then
    die "Build completed but no firmware artifacts (.uf2/.elf/.bin/.hex/.map) were produced"
  fi
  warn "No firmware artifacts found for target '${TARGET}'"
fi

artifacts_dir="${BUILD_DIR}"
if [[ ${STAGE} -eq 1 ]]; then
  artifacts_dir="${BUILD_DIR}/artifacts"
  if [[ ${DRY_RUN} -eq 0 ]]; then
    rm -rf -- "${artifacts_dir}"
    mkdir -p -- "${artifacts_dir}"
    for file in "${artifacts[@]}"; do
      cp -f -- "${file}" "${artifacts_dir}/"
    done

    git_commit="$(git -C "${REPO_ROOT}" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    git_dirty=no
    if [[ -n "$(git -C "${REPO_ROOT}" status --porcelain --untracked-files=no 2>/dev/null)" ]]; then
      git_dirty=yes
    fi
    {
      printf 'build_date=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
      printf 'git_commit=%s\n' "${git_commit}"
      printf 'git_dirty=%s\n' "${git_dirty}"
      printf 'board=%s\n' "${BOARD}"
      printf 'build_type=%s\n' "${BUILD_TYPE}"
      printf 'sdk=%s\n' "${SDK}"
      printf 'toolchain=%s\n' "${TOOLCHAIN}"
      printf 'target=%s\n' "${TARGET}"
      printf 'artifacts:\n'
      for file in "${artifacts[@]}"; do
        printf '  %s %s bytes\n' "$(basename -- "${file}")" "$(stat -c %s -- "${file}")"
      done
    } > "${artifacts_dir}/manifest.txt"
  fi
fi

# Report --------------------------------------------------------------------
if [[ ${DRY_RUN} -eq 1 ]]; then
  log "Dry run complete"
  exit 0
fi

log "Artifacts (${artifacts_dir})"
for file in "${artifacts_dir}"/*; do
  [[ -f "${file}" ]] || continue
  printf '    %-32s %10s\n' "$(basename -- "${file}")" "$(human_size "$(stat -c %s -- "${file}")")"
done

uf2=""
for file in "${artifacts_dir}"/*.uf2; do
  [[ -f "${file}" ]] || continue
  uf2="${file}"
  break
done
if [[ -n "${uf2}" ]]; then
  printf '    Flash: hold BOOTSEL, connect USB, copy the UF2 to the RPI-RP2 drive\n'
  printf '           %s\n' "${uf2}"
fi

success "Build complete in ${SECONDS}s"
