# uav_gazebo_environments

This repository contains the Gazebo worlds used in the UAV exploration and 3D reconstruction
experiments of [UAV_3D_reconstruction](https://github.com/joaom2a0r0i1a/UAV_3D_reconstruction),
where it is included as a submodule. For each world, `config/<environment>.yaml` gives the spawn
position and the planning, gain and reconstruction regions used by the planners and the
evaluation.

## Worlds

| world | what it is | who made it |
|---|---|---|
| `grass_plane_multistory.world` | 3 floor office, box primitives | mine, from `scripts/gen_multistory.py` |
| `grass_plane_warehouse.world` | racking, pallets and a loading dock | mine, from `scripts/gen_warehouse.py` |
| `big_maze.world` | large maze, box primitives | mine |
| `grass_plane_school.world` | school on a grass plane | arrangement mine, `school` model is not |
| `grass_plane_police_station.world` | police station on a grass plane | arrangement mine, `police_station` model is not |
| `grass_plane_maze.world` | maze plus two closing walls | arrangement mine, `maze` and `grey_wall` models are not |

The multistory, the warehouse and the big maze are my own geometry. The other three worlds place
an existing model on a grass plane.

## Model credits

The models below are not mine. They are included so the worlds load without extra downloads, and
each one belongs to its author.

| model | author | taken from |
|---|---|---|
| `school` | Nate Koenig, OSRF | [`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources), BSD 3-Clause |
| `police_station` | Nate Koenig, OSRF | the Gazebo model database |
| `grey_wall` | Maurice Fallon | [`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources), BSD 3-Clause |
| `grass_plane` | Petr Stibinger, CTU MRS | [`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources), BSD 3-Clause |
| `maze` | Zhefan Xu, CMU | [`Zhefan-Xu/drone_gazebo`](https://github.com/Zhefan-Xu/drone_gazebo) |

The maze comes from a repository without a license file. It is included for the reproducibility
of the experiments and credited to its author, who can ask for its removal by opening an issue.

## Use

The worlds come with UAV_3D_reconstruction. They can also be used on their own in a catkin
workspace:

```bash
cd <catkin_ws>/src
git clone git@github.com:joaom2a0r0i1a/uav_gazebo_environments.git
cd .. && catkin build uav_gazebo_environments && source devel/setup.bash
```

A world is then launched with:

```bash
roslaunch gazebo_ros empty_world.launch \
  world_name:=$(rospack find uav_gazebo_environments)/worlds/grass_plane_school.world
```

The model paths come from the `gazebo_ros` export in `package.xml`, and `env-hooks/` sets the same
paths for shells that run `gzserver` directly.

## Running without MRS

Every world loads two plugins from
[`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources):

```xml
<plugin name='mrs_gazebo_static_transform_republisher_plugin'
        filename='libMrsGazeboCommonResources_StaticTransformRepublisher.so'/>
<plugin name='mrs_gazebo_rviz_cam_synchronizer'
        filename='libMrsGazeboCommonResources_RvizCameraSynchronizer.so'>
```

The static transform republisher provides the transforms used by the mapping and planning nodes,
and the camera synchroniser keeps an RViz view aligned with a Gazebo camera. Without MRS, Gazebo
reports that it cannot load them and opens the world normally.

To use a world without MRS, remove both plugin lines from it and publish the static transforms
yourself. Nothing else in the worlds depends on MRS.

## Checking the worlds

```bash
scripts/check_worlds.sh            # all worlds, headless
scripts/check_worlds.sh --gui      # with the Gazebo window
```

Each world is launched on its own Gazebo master and counts as loaded when
`/gazebo/get_world_properties` answers. The script also checks that every model file used by the
world exists, since a model with a missing mesh still loads but has no geometry. All worlds report
a case mismatch for `Gazebo.material`, which is harmless.

## Ground truth

`ground_truth/<environment>.ply` holds the reference cloud the reconstruction is scored against,
for `school` and `police`. The warehouse, multistory and big maze are evaluated by volume.

## Generators

`scripts/gen_multistory.py` and `scripts/gen_warehouse.py` generate their world file and a ground
truth cloud, written to `ground_truth/` or to the path in `GT_OUT`. Both reproduce their world
exactly. The multistory cloud has the same geometry on every run, but about 6000 of its 1.29 M
points change colour between runs, which does not affect the evaluation. The big maze has no
generator.
