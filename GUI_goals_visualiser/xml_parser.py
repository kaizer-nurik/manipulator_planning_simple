"""Чтение XML файла"""


from robot_class import Robot_class
from obstacles import ObstacleManager
from goal_point import GoalPoint
import xml.etree.ElementTree as ET
from PySide6.QtCore import QPointF
from PySide6.QtGui import QPolygonF
import numpy as np


def read_xml(filename: str,scene) -> (str, Robot_class, ObstacleManager, GoalPoint):
    """Чтение xml файла filename"""
    robot: Robot_class = Robot_class()
    robot.reset()
    obstacles: ObstacleManager = ObstacleManager(None, None, None, scene)
    obstacles.reset()

    goal_point: GoalPoint = GoalPoint(10, scene)
    goal_point.reset()

    tree = ET.parse(filename)

    root = tree.getroot()
    robot_info = root.find('robot_info')
    joints = robot_info.find('joints')
    start_configuration = root.find('start_configuration')
    for joint, joint_info, st_conf in zip(robot, joints, start_configuration):
        joint.update_left_limit(float(joint_info.get('limit_min')))
        joint.update_start_angle(float(st_conf.text))
        joint.update_right_limit(float(joint_info.get('limit_max')))
        joint.update_length(float(joint_info.get('length'))*100)
        joint.rotate(float(st_conf.text))

        robot.add_joint()

    robot.pop_joint()

    goal_info = root.find('goal_point')

    goal_point.set_angle((float(goal_info.get('angle'))-180)*np.pi/180)
    goal_point.create_dot(
        QPointF(float(goal_info.get('x'))*100, float(goal_info.get('y'))*100))
    # try:
    #     goal_point.update_radius(float(goal_info.get('delta_radius'))*100)
    # except BaseException as e:
    #     print(e)

    scene_info = root.find('scene')

    for polygon in scene_info.findall('polygon'):
        obstacles.add_obstacle()
        poli = obstacles.obstacles[-1]
        poli.set_dots_visible(False)
        for vertex in polygon.findall('vertex'):
            poli.add_dot(QPointF(float(vertex.get('x')) *
                         100, float(vertex.get('y'))*100))
        poli.set_dots_visible(False)

    csv_xml = root.find('csv')
    if csv_xml is None:
        csv_xml = ""
    else:
        csv_xml = csv_xml.text
    return (csv_xml, robot, obstacles, goal_point)
