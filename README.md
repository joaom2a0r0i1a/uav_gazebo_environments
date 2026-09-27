# Environments

Gazebo worlds and models for the UAV exploration and 3D reconstruction experiments.

The package is standalone: it does not depend on the MRS stack. Everything a world needs is
either in `models/` or ships with Gazebo, with one exception, the maze, which is fetched.

## Worlds

| world | contents | source |
|---|---|---|
| `grass_plane_school.world` | school building on a grass plane | `school` model |
| `grass_plane_police_station.world` | police station | `police_station` model |
| `grass_plane_maze.world` | maze plus two closing walls | fetched `maze` model |
| `big_maze.world` | large maze, inline box geometry | self-contained |
| `grass_plane_multistory.world` | 3 floor office | `gen_multistory.py` |
| `grass_plane_warehouse.world` | racking and loading dock | `gen_warehouse.py` |

## Use

Clone into a catkin workspace and build:

```bash
cd <catkin_ws>/src
git clone git@github.com:joaom2a0r0i1a/uav_gazebo_environments.git
cd .. && catkin build environments && source devel/setup.bash
```

Then launch a world:

```bash
roslaunch gazebo_ros empty_world.launch \
  world_name:=$(rospack find environments)/worlds/grass_plane_school.world
```

Model paths come from the `<gazebo_ros>` export tags in `package.xml`, which
`gazebo_ros_paths_plugin` applies inside gzserver. `env-hooks/` sets the same paths for plain
shells that run `gzserver` without roslaunch.

## The maze

`grass_plane_maze.world` needs a mesh from
[Zhefan-Xu/drone_gazebo](https://github.com/Zhefan-Xu/drone_gazebo), which carries no license
file. It is fetched rather than redistributed, and `models/maze/` is gitignored:

```bash
scripts/fetch_maze_model.sh
```

## Checking the worlds

```bash
scripts/check_worlds.sh            # all worlds, headless
scripts/check_worlds.sh --gui      # watch them load
```

Each world is launched on its own Gazebo master port, and readiness comes from
`/gazebo/get_world_properties`. A model whose mesh is missing still spawns as a named entity
with no geometry, so the script also resolves every asset URI and reports the ones that do not
exist. All six worlds report `Gazebo.material` as case-only: the world files spell it with a
capital G and the file on disk is `gazebo.material`. It is harmless and predates this package.

## Generators

`worlds/gen_multistory.py` and `worlds/gen_warehouse.py` emit their world file and an analytic
ground truth cloud. Both are seeded and reproduce their world byte for byte. The cloud goes to
the evaluation package by default; set `GT_OUT` to write it elsewhere.

There is no generator for `big_maze.world` or for the fetched `maze`.

## Attribution

`police_station` is ours. `grass_plane`, `grey_wall` and `school` are vendored from
[`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources)
(BSD 3-Clause); see `models/README.md` for the original authors. This package is BSD 3-Clause,
see `LICENSE`.
