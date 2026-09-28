# uav_gazebo_environments

Gazebo worlds used for UAV exploration and 3D reconstruction experiments.

It is the `uav_gazebo_environments` submodule of
[UAV_3D_reconstruction](https://github.com/joaom2a0r0i1a/UAV_3D_reconstruction), which pins
the commit it uses. Clone that repository with `--recursive` rather than this one on its own.
`config/<environment>.yaml` holds each world's spawn and its planning, gain and reconstruction
regions, read by the planners and the evaluation there.

## Worlds

| world | what it is | who made it |
|---|---|---|
| `grass_plane_multistory.world` | 3 floor office, box primitives | mine, from `scripts/gen_multistory.py` |
| `grass_plane_warehouse.world` | racking, pallets and a loading dock | mine, from `scripts/gen_warehouse.py` |
| `big_maze.world` | large maze, box primitives | mine |
| `grass_plane_school.world` | school on a grass plane | arrangement mine, `school` model is not |
| `grass_plane_police_station.world` | police station on a grass plane | arrangement mine, `police_station` model is not |
| `grass_plane_maze.world` | maze plus two closing walls | arrangement mine, `maze` and `grey_wall` models are not |

Only the multistory, the warehouse and the big maze are my own geometry. The other three
worlds are arrangements that place an existing model on a grass plane.

## Model credits

None of the models below are mine. They are included so the worlds load without chasing
downloads, and each one belongs to its author.

| model | author | taken from |
|---|---|---|
| `school` | Nate Koenig, OSRF | [`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources), BSD 3-Clause |
| `police_station` | Nate Koenig, OSRF | the Gazebo model database |
| `grey_wall` | Maurice Fallon | [`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources), BSD 3-Clause |
| `grass_plane` | Petr Stibinger, CTU MRS | [`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources), BSD 3-Clause |
| `maze` | Zhefan Xu, CMU | [`Zhefan-Xu/drone_gazebo`](https://github.com/Zhefan-Xu/drone_gazebo) |

The maze comes from a repository that carries **no license file**, so no terms were granted
with it. It is kept here for reproducibility of the experiments and credited to its author.
If you are the author and would rather it were not redistributed, open an issue and it will
be removed.

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

## Running without the MRS stack

Every world loads two plugins from
[`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources):

```xml
<plugin name='mrs_gazebo_static_transform_republisher_plugin'
        filename='libMrsGazeboCommonResources_StaticTransformRepublisher.so'/>
<plugin name='mrs_gazebo_rviz_cam_synchronizer'
        filename='libMrsGazeboCommonResources_RvizCameraSynchronizer.so'>
```

They exist for the MRS simulation setup. The static transform republisher feeds the TF tree
that the mapping and planning nodes expect, and the camera synchroniser keeps an RViz view
aligned with a Gazebo camera. Without MRS installed, Gazebo prints a failure to load each one
and carries on, so the world still opens and the geometry is all there.

If you are not running MRS, delete both `<plugin>` lines from the world you want and supply
your own static transforms. Nothing else in these worlds depends on MRS.

## Checking the worlds

```bash
scripts/check_worlds.sh            # all worlds, headless
scripts/check_worlds.sh --gui      # watch them load
```

Each world is launched on its own Gazebo master port and readiness comes from
`/gazebo/get_world_properties`. A model whose mesh is missing still spawns as a named entity
with no geometry, so the script also resolves every asset URI and reports the ones that do not
exist. All six worlds report `Gazebo.material` as a case mismatch, because the world files
spell it with a capital G while the file on disk is `gazebo.material`. It is harmless.

## Generators

`scripts/gen_multistory.py` and `scripts/gen_warehouse.py` write their world file and an
analytic ground truth cloud. Both reproduce their world byte for byte. The cloud goes to the
evaluation package of the parent repository, which holds this one as its `Environments`
submodule, and `GT_OUT` sends it elsewhere.

`gen_multistory.py` is not fully reproducible in the cloud it writes. Geometry, normals and
indices come out bit identical every run, but about 6000 of its 1.29 M points change colour
between runs. Colour is not used by the evaluation.

There is no generator for `big_maze.world`, and the maze and the other imported models have
none by nature.
