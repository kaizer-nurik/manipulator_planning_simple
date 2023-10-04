#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include "rapidxml.hpp"
#include "obstacles.h"
#include "robot.h"


bool read_scene(const std::string &filename, std::vector<Polygon> &polygons, Robot &start, GoalPoint &goal);