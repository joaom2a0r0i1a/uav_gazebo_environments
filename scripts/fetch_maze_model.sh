#!/bin/bash
# Fetch the maze model used by grass_plane_maze.world.
#
# The mesh comes from Zhefan-Xu/drone_gazebo, which carries no license file, so it is fetched
# here rather than redistributed with this package. models/maze/ is gitignored for that reason.
set -e

DEST="$(cd "$(dirname "$0")/.." && pwd)/models/maze"
REPO=https://github.com/Zhefan-Xu/drone_gazebo
RAW=https://raw.githubusercontent.com/Zhefan-Xu/drone_gazebo/master/models/maze

if [ -e "$DEST/meshes/model_maze.dae" ]; then
  echo "maze model already present at $DEST"
  exit 0
fi

echo "fetching the maze model from $REPO"
mkdir -p "$DEST/meshes"
for f in model.config model.sdf; do
  curl -fSL "$RAW/$f" -o "$DEST/$f"
done
# model_maze.dae is what the world references; model.dae is what upstream model.sdf uses
for f in model_maze.dae model.dae; do
  curl -fSL "$RAW/meshes/$f" -o "$DEST/meshes/$f"
done

echo "done: $DEST"
echo "verify with: scripts/check_worlds.sh worlds/grass_plane_maze.world"
