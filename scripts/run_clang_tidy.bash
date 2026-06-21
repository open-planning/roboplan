#!/bin/bash

# Helper script to run clang-tidy on one or all packages in the repository.
# Uses run-clang-tidy (from clang-tools) for parallel execution.

set -e

# Calculate paths
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_ROOT_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"

# Default values
PACKAGE_NAME=""
FIX_FLAG=""
EXIT_CODE=0

# Discover packages dynamically: subdirs at repo root with a CMakeLists.txt, excluding external/
mapfile -t ALL_PACKAGES < <(
  find "${REPO_ROOT_DIR}" -maxdepth 2 -mindepth 2 -name "CMakeLists.txt" \
    -not -path "${REPO_ROOT_DIR}/external/*" \
    -not -path "${REPO_ROOT_DIR}/build/*" \
    | xargs -I{} dirname {} \
    | xargs -I{} basename {} \
    | sort
)

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --fix)
      FIX_FLAG="-fix"
      shift
      ;;
    --help)
      echo "Usage: $0 [<package>] [--fix]"
      echo ""
      echo "Arguments:"
      echo "  <package>  Run clang-tidy on specific package (optional)"
      echo ""
      echo "Options:"
      echo "  --fix      Apply clang-tidy fixes automatically"
      echo "  --help     Show this help message"
      echo ""
      echo "Valid package names: ${ALL_PACKAGES[*]}"
      echo ""
      echo "Examples:"
      echo "  $0                    # Check all packages"
      echo "  $0 roboplan           # Check roboplan package only"
      echo "  $0 roboplan --fix     # Check and fix roboplan package"
      echo "  $0 --fix              # Check and fix all packages"
      exit 0
      ;;
    *)
      # Skip empty arguments (from pixi when package_name is not provided)
      if [[ -z "$1" ]]; then
        shift
      # Treat as package name if not empty
      elif [[ -z "$PACKAGE_NAME" ]]; then
        PACKAGE_NAME="$1"
        shift
      else
        echo "Unknown option or duplicate package name: $1"
        echo "Run '$0 --help' for usage information"
        exit 1
      fi
      ;;
  esac
done

# Validate package name if provided
if [[ -n "$PACKAGE_NAME" ]]; then
  if [[ ! -d "${REPO_ROOT_DIR}/${PACKAGE_NAME}" ]]; then
    echo "Error: Unknown package '${PACKAGE_NAME}'"
    echo "Valid package names: ${ALL_PACKAGES[*]}"
    exit 1
  fi
  PACKAGES=("$PACKAGE_NAME")
else
  PACKAGES=("${ALL_PACKAGES[@]}")
fi

pushd "${REPO_ROOT_DIR}" > /dev/null || exit

for PKG in "${PACKAGES[@]}"; do
  BUILD_DIR="${REPO_ROOT_DIR}/build/${PKG}"
  COMPILE_DB="${BUILD_DIR}/compile_commands.json"

  # Check if compilation database exists
  if [[ ! -f "$COMPILE_DB" ]]; then
    echo "========================================================================"
    echo "Warning: Skipping ${PKG} - compilation database not found"
    echo "         Expected: ${COMPILE_DB}"
    echo "         Run 'pixi run build --package_name ${PKG}' first"
    echo "========================================================================"
    echo ""
    continue
  fi

  echo "========================================================================"
  echo "Running clang-tidy on ${PKG}..."
  echo "========================================================================"

  # Run clang-tidy using the run-clang-tidy script (parallel execution)
  # This checks:
  # - All source files (.cpp, .c) from compile_commands.json
  # - Headers included by those source files (via HeaderFilterRegex in .clang-tidy)
  # Third-party code is excluded via .clang-tidy HeaderFilterRegex pattern
  if ! run-clang-tidy \
    -p "${BUILD_DIR}" \
    ${FIX_FLAG} \
    -quiet \
    "^${REPO_ROOT_DIR}/[^.]"; then
    EXIT_CODE=1
    echo ""
    echo "clang-tidy found issues in ${PKG}"
  fi

  echo ""
done

popd > /dev/null || exit

if [[ $EXIT_CODE -ne 0 ]]; then
  echo "========================================================================"
  echo "clang-tidy failed!"
  echo "========================================================================"
else
  echo "========================================================================"
  echo "clang-tidy passed!"
  echo "========================================================================"
fi

exit $EXIT_CODE
