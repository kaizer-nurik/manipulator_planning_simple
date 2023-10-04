"""Создание XML файла по введённым в сцену объектам"""


from robot_class import Robot_class
from obstacles import ObstacleManager
from goal_point import GoalPoint
import xml.etree.ElementTree as ET

GOAL_POINT_TRESHOLD = 10 #абсолютная погрешность угла конечной точки

def to_xml(filename: str, robot: Robot_class, obstacles: ObstacleManager, goal_point: GoalPoint):
    """Создание xml файла filename"""
    root = ET.Element('input_info')
    add_robot_info(root,robot)# Добавления информации о роботе
    add_start_conf(root,robot)# Добавление информации о начальной конфигурации
    add_goal_point(root,goal_point)# Добавление информации о цели
    add_scene(root, obstacles)
    
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ", level=0)
    tree.write(filename,encoding='unicode',xml_declaration = True)

def add_scene(root:ET.Element, obstacle: ObstacleManager):    
    scene = ET.SubElement(root,"scene")
    
    for polygon in obstacle:
        polygon_xml = ET.SubElement(scene, "polygon")
        for vertex in polygon:
            vertex_xml = ET.SubElement(polygon_xml, "vertex")
            vertex_xml.set('x', str(vertex.x()/100))
            vertex_xml.set('y', str(vertex.y()/100))
           
            
    
def add_goal_point(root:ET.Element, goal_point:GoalPoint):
    goal_point_xml = ET.SubElement(root,'goal_point')
    goal_point_xml.set('x', str(goal_point.x()/100))
    goal_point_xml.set('y', str(goal_point.y()/100))
    goal_point_xml.set('delta_radius', str(goal_point.radius/100))
    goal_point_xml.set('angle', str(((goal_point.angle()))% 360 - 180 ))
    goal_point_xml.set('angle_tolerance', str(GOAL_POINT_TRESHOLD) )
    
    
    

def add_start_conf(root:ET.Element,robot:Robot_class):    
    start_configuration = ET.SubElement(root,'start_configuration')
    for joint_number, joint in enumerate(robot):
        angle = ET.SubElement(start_configuration,"angle")
        angle.set('number', str(joint_number))
        angle.text = str(joint.start_angle)
        
                
def add_robot_info(root:ET.Element,robot:Robot_class):
    robot_info = ET.SubElement(root,'robot_info')
    joints = ET.SubElement(robot_info,'joints')
    
    for joint_number, robot_joint in enumerate(robot):
        joint = ET.SubElement(joints,'joint')
        joint.set('number', str(joint_number))
        joint.set('length', str(robot_joint.length/100))
        joint.set('width',str(robot_joint.width/100))
        joint.set('limit_min',str(robot_joint.left_angle))
        joint.set('limit_max',str(robot_joint.right_angle))
