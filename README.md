# uav_gazebo_environments

This repository contains the Gazebo worlds used in the UAV exploration and 3D reconstruction
experiments of [UAV_3D_reconstruction](https://github.com/joaom2a0r0i1a/UAV_3D_reconstruction),
where it is included as a submodule. For each world, `config/<environment>.yaml` gives the spawn
position and the planning, gain and reconstruction regions.

## Worlds

| world | what it is | origin |
|---|---|---|
| `grass_plane_multistory.world` | 3 floor office, boxes | made for this repository |
| `grass_plane_warehouse.world` | racking, pallets and a loading dock | made for this repository |
| `big_maze.world` | large maze | made for this repository |
| `grass_plane_school.world` | school on a grass plane | made for this repository, `school` model credited below |
| `grass_plane_police_station.world` | police station on a grass plane | made for this repository, `police_station` model credited below |
| `grass_plane_maze.world` | maze plus two closing walls | made for this repository, `maze` and `grey_wall` models credited below |

The multistory, the warehouse and the big maze were made for this repository. The other three
worlds place an existing model on a grass plane.

## Model credits

The models below belong to their authors and are included so the worlds load without extra
downloads.

| model | author | taken from |
|---|---|---|
| `school` | Nate Koenig, OSRF | [`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources), BSD 3-Clause |
| `police_station` | Nate Koenig, OSRF | the Gazebo model database |
| `grey_wall` | Maurice Fallon | [`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources), BSD 3-Clause |
| `grass_plane` | Petr Stibinger, CTU MRS | [`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources), BSD 3-Clause |
| `maze` | Zhefan Xu, CMU | [`Zhefan-Xu/drone_gazebo`](https://github.com/Zhefan-Xu/drone_gazebo) |

The maze comes from a repository without a license file. It is included to reproduce the
experiments, but can be removed if its author asks for it.

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

The static transform republisher publishes the transforms used by the mapping and planning nodes,
and the camera synchroniser aligns an RViz view with a Gazebo camera. Without MRS, both plugins
fail to load and the world still opens.

To use a world without MRS, remove both plugin lines from it and publish the static transforms
yourself. Nothing else in the worlds depends on MRS.

## Checking the worlds

```bash
scripts/check_worlds.sh            # all worlds, headless
scripts/check_worlds.sh --gui      # with the Gazebo window
```

Each world is launched on its own Gazebo master and counts as loaded when
`/gazebo/get_world_properties` answers. The script also checks that every model file used by the
world exists. The case mismatch reported for `Gazebo.material` is harmless.

## Ground truth

`ground_truth/<environment>.ply` holds the reference cloud the reconstruction is scored against,
for `school` and `police`. The warehouse, multistory and big maze are evaluated by volume.

## Generators

`scripts/gen_multistory.py` and `scripts/gen_warehouse.py` generate their world file and a ground
truth cloud in `ground_truth/`. `GT_OUT` sets another path for the cloud. Both reproduce their
world exactly. The multistory cloud keeps the same geometry on every run, only the colour of about
6000 of its 1.29 M points changes. The big maze has no generator.
