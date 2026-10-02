#!/usr/bin/env bash
# Sync Windows-edited package files into the WSL colcon workspace, then rebuild.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$(cd "${SCRIPT_DIR}/.." && pwd)"
DST="${HOME}/41068_ws/src/41068_ignition_bringup"

echo "=== sync_to_wsl ==="
echo "  from: ${SRC}"
echo "  to:   ${DST}"

mkdir -p "${DST}"/{models,worlds,launch,scripts,config,urdf_husky,urdf_parrot,maps}
mkdir -p "${SRC}/maps"

# Models + URDF may contain nested mesh folders.
cp -r "${SRC}/models/." "${DST}/models/"
# Nested snow_props/ was promoted to top-level models/snow_*; drop stale copy.
rm -rf "${DST}/models/snow_props"
cp "${SRC}/worlds/"* "${DST}/worlds/"
cp "${SRC}/launch/"*.py "${DST}/launch/"

# Copy only source scripts — never __pycache__ or other dirs.
shopt -s nullglob
for f in "${SRC}/scripts/"*.py "${SRC}/scripts/"*.sh; do
  cp "${f}" "${DST}/scripts/"
  chmod +x "${DST}/scripts/$(basename "${f}")" 2>/dev/null || true
done

rm -rf "${DST}/scripts/__pycache__" "${DST}/launch/__pycache__"

cp "${SRC}/config/"* "${DST}/config/"
cp -r "${SRC}/urdf_husky/." "${DST}/urdf_husky/"
cp -r "${SRC}/urdf_parrot/." "${DST}/urdf_parrot/"
cp -r "${SRC}/maps/." "${DST}/maps/" 2>/dev/null || true
cp -r "${DST}/maps/." "${SRC}/maps/" 2>/dev/null || true
cp "${SRC}/CMakeLists.txt" "${SRC}/package.xml" "${DST}/"

echo "  synced: models worlds launch scripts config urdf_husky urdf_parrot maps CMakeLists package.xml"

# Quick sanity: Phase-3 ice cable assets must be present after sync.
missing=0
for rel in \
  worlds/snow_ice_cable_test.sdf \
  models/ice_block_medium/model.sdf \
  urdf_parrot/parrot.urdf.xacro \
  config/41068_parrot1.rviz
do
  if [[ ! -e "${DST}/${rel}" ]]; then
    echo "ERROR: missing after sync: ${DST}/${rel}" >&2
    missing=1
  fi
done
if [[ "${missing}" -ne 0 ]]; then
  exit 1
fi

cd "${HOME}/41068_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select 41068_ignition_bringup

echo "=== sync_to_wsl OK ==="
