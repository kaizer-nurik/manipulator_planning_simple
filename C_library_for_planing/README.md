# С++ Library for manipulator path planning

# Quickstart

### 1. Build the code
1) git clone this repo


```
git clone https://github.com/kaizer-nurik/manipulator_planning_simple.git
```

2) Change active directory to this

```
cd ./manipulator_planning_simple/C_library_for_planing
```

3) Create build directory and go into it

```
mkdir ./build
cd build
```

4) Execute Cmakefile

```
cmake ..
```
5) Build the code!

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

# Структура входного XML файла:
```
<?xml version="1.0" encoding="UTF-8"?>
<input_info>
  <robot_info> - Информация о роботе
    <joints>   - Перечисление информации о каждом звене робота, можно задавать хоть сколько звеньев
      <joint>
        <length> length </length> длина звена (в метрах)
        <width>  width  </width>  ширина звена (в метрах)
        <limit1> limit  </limit1> ограничение по углу поворота относительно предыдущего звена (в градусах)
        <limit2> limit  </limit2> ограничение по углу поворота относительно предыдущего звена (в градусах)
        </joint>
        .
        .
        .

    </joints>
  </robot_info>

  <start_configuration> Информация о стартовой конфигурации: задаются углы поворота
    <angle> angle1 </angle> каждого соединения, конфигурация робота задается как вектор
    <angle> angle2 </angle> длины DOF из углов 
    .   Без потери общности считаем, что начало манипулятора находится в (0, 0)       
    .   примечание: в GUI начало (500, 500)
    .

  </start_configuration>

  <goal_point>  Информация о точке назначения: координаты точки, куда должен добраться
    <x> x </x> дефлектор манипулятора
    <y> y </y>
    <angle1> angle1 </angle1> angle1, angle2 задают сектор, в котором последнее звено манипулятора
    <angle2> angle2 </angle2> должно приблизиться к точке goal
  </goal_point>

  <scene> Информация о сцене с препятствиями
    <polygon> Препятствия представляют собой многоугольники, для каждого многоугольника можно задать
      <vertex> его количество вершин (треугольник, четырехугольник, ...)
        <x> 1 </x> Каждая вершина характеризуется координатами на плоскости (в пространстве)
        <y> 1 </y>
      </vertex>
      .
      .
      . 
    </polygon>
    . Задается много полигонов (многоугольников)
    .
    .
  </scene>
</input_info>
```
