# С++ Library for manipulator path planning

# Quickstart

### 1. Build the code
1) git clone this repo


```
git clone https://github.com/kaizer-nurik/manipulator_planning_simple.git
```

2) init submodules

```
git submodule init

git submodule update
```

3) Change active directory to this

```
cd ./manipulator_planning_simple/C_library_for_planing
```

4) Create build directory and go into it

```
mkdir ./build
cd build
```

5) Execute Cmakefile

```
cmake ..
```
6) Build the code!

```
make
```

### 2. Execute the code

1) To execute RRT in single query mode use `./main_rrt_single` executable in ./build directory.

```
./main_rrt_single [angle_step] [goal_bias] [max_iteration_count] [path_to_file]
```

#### Arguments:

angle_step: size of rrt step in configuration space of manipulator. float

goal_bias: probability of expansion towards goal

max_iteration_count: maximum iterations, after exceeding this rrt will fail. long

path_to_file: path to xml file with task content. string

***
#### Returns
Results will be written in:

`result_trajectory.csv` - csv file  with sequence of manipulator positions, separator ','

`result_trajectory.xml` - input xml file with <csv></csv> tag, containing path sequence as csv file.

`rrt_tree.csv` - csv file with RRT tree, containing node_id, config1, config2, parent_id. Saves only 2 configuration coordinates. separator ','

`results_IK_res.csv` - csv file with array of configurations, that are Inverse Kinematic solution for given goal

`results_stats.json` - json file with execution stats
time is in nanoseconds.

```
{
"angle_step":"10.000000",
"goal bias":"0.100000",
"number_of_IK_results":"800",
"number_of_collision_check":"10006",
"number_of_collision_check_in_IK":"800",
"number_of_denied_nodes_goal":"981",
"number_of_denied_nodes_random":"2085",
"number_of_goal_expanding_nodes":"6",
"number_of_nn_check":"0",
"number_of_nodes":"6941",
"number_of_random_nodes":"6934",
"reached_goal":"0",
"time_of_IK_results":"5154588.000000",
"time_of_collision_check":"32082306.000000",
"time_of_collision_check_in_IK":"2432525.000000",
"time_of_nn_check":"61989155.000000",
"timeout":"0",
"total_duration":"177653080"
}

```


for example, in ./build directory:
```
./main_rrt_single 10.0 0.1 10000 ../../dataset/scene1/scene_1_test_1.xml
```

___

2) to execute A* in single query mode use `./main_a_star_single` executable in ./build directory

```
./main_a_star_single [path_to_file]
```
#### Arguments:

path_to_file: path to file, string

for example, in ./build directory:
```
./main_a_star_single ../../dataset/scene1/scene_1_test_1.xml
```

#### Returns
Results will be written in

`result_trajectory.xml` - input xml file with <csv></csv> tag, containing path sequence as csv file.

`scene1_0_1_data.json` - json file with execution stats
time is in seconds.

```
{
    "_number": 0,
    "closed_nodes": 66,
    "coll_check_percentage": 73.933421262039,
    "coord_tolerance": 0.1,
    "g_cost": 6.062317824099973,
    "g_units": 30,
    "opened_nodes": 1832,
    "total_time": 0.018050798,
    "turn_number": 65
}
```

___


3) To execute RRT in dataset mode use `./main_rrt` executable in ./build directory. Folder 'dataset' must be copied in ./build directory. Note, that each test will be executed 100 times.

```
./main_rrt [angle_step] [goal_bias] [max_iteration_count] [dataset_code]
```
#### Arguments:

angle_step: size of rrt step in configuration space of manipulator. float

goal_bias: probability of expansion towards goal

max_iteration_count: maximum iterations, after exceeding this rrt will fail. long

dataset_code: dataset code of test, string

dataset_code may be:

1 - for the scene 1, manual tests
1r - for the scene 1, random tests
2 - for the scene 2, manual tests
2r - for the scene 2, random tests
3 - for the scene 3, manual tests
3r - for the scene 3, random tests

***
#### Returns
Results will be written in the folder ./results_of_scene{i}#{j} where
i - dataset_code
j - number of iteration.

This folder will contain same files as single query mode, but for all tests.

for example, copy dataset folde in ./build directory and execute:
```
./main_rrt 10.0 0.1 10000 1r
```

___

4) to execute A* in dataset mode use `./main_a_star` executable in ./build directory. Folder 'dataset' must be copied in ./build directory.

```
./main_a_star [path_to_file]
```
#### Arguments:

dataset_code: dataset code of test, string

dataset_code may be:

1 - for the scene 1, manual tests
1r - for the scene 1, random tests
2 - for the scene 2, manual tests
2r - for the scene 2, random tests
3 - for the scene 3, manual tests
3r - for the scene 3, random tests

for example, in ./build directory:
```
./main_a_star 1r
```

#### Returns
Results will be written

`scene_{i_random}_test_{j}_trajectory.xml.xml` - input xml file with <csv></csv> tag, containing path sequence as csv file.
{i_random}, {i} - dataset_code
{j} - number of test

`scene1_0_1_data.json` - json file with execution stats of all tests
time is in seconds.

```
{
    "_number": 0,
    "closed_nodes": 66,
    "coll_check_percentage": 73.933421262039,
    "coord_tolerance": 0.1,
    "g_cost": 6.062317824099973,
    "g_units": 30,
    "opened_nodes": 1832,
    "total_time": 0.018050798,
    "turn_number": 65
}
```

___

# Structure of the input XML file:
```
<?xml version='1.0' encoding='utf-8'?>
<input_info>
  <robot_info> Information about the manipulator
    <joints> information about the manipulator joints: joint's number, joint's length and width in meters, limits of joint rotation in degrees relative to the previous joint
      <joint number="0" length="1.0" width="0.2" limit_min="-180.0" limit_max="180.0" />
      <joint number="1" length="1.0" width="0.2" limit_min="-180.0" limit_max="180.0" />
      <joint number="2" length="1.0" width="0.2" limit_min="-180.0" limit_max="180.0" />
      <joint number="3" length="1.0" width="0.2" limit_min="-180" limit_max="180" />
    </joints>
  </robot_info>
  <start_configuration> Information about the starting configuration of the manipulator: each joint's angle in degrees
    <angle number="0">87.70938995736145</angle>
    <angle number="1">2.087382155281091</angle>
    <angle number="2">2.4627368642812257</angle>
    <angle number="3">-10.487852845001214</angle>
  </start_configuration> Cartesian point and coordinate tolerance in meters where the end-effector of the manipulator should arrive  and the orientation of the last joint with
  <goal_point x="1.75" y="3.6111111111111107" coord_tolerance="0.1" angle="53.130102354155895" angle_tolerance="10" /> an angle tolerance in degrees
  <scene> Scene information: sequence of obstacles in the form of polygons
    <polygon> each polygon is specified as a set of vertices, the coordinates of the vertices are specified in meters
      <vertex x="-2.7944444444444447" y="-0.85" />
      <vertex x="-2.7388888888888885" y="-2.683333333333333" />
      <vertex x="-0.9055555555555556" y="-2.6" />
      <vertex x="-1.35" y="-1.2944444444444445" />
    </polygon>
    <polygon>
      <vertex x="-3.433333333333333" y="3.3722222222222222" />
      <vertex x="-3.016666666666666" y="1.5388888888888888" />
      <vertex x="-1.988888888888889" y="2.15" />
      <vertex x="-1.7944444444444443" y="3.483333333333333" />
    </polygon>
    <polygon>
      <vertex x="1.761111111111111" y="2.65" />
      <vertex x="1.761111111111111" y="1.6777777777777778" />
      <vertex x="3.1222222222222222" y="1.3722222222222222" />
      <vertex x="3.0944444444444446" y="2.733333333333333" />
    </polygon>
    <polygon>
      <vertex x="2.4277777777777776" y="-1.35" />
      <vertex x="1.7055555555555555" y="-2.8777777777777778" />
      <vertex x="4.816666666666666" y="-2.9055555555555554" />
      <vertex x="4.777777777777778" y="-1.3611111111111112" />
    </polygon>
  </scene>
</input_info>
```
