"""Чтение XML файла"""


from robot_class import Robot_class
from obstacles import ObstacleManager
from goal_point import GoalPoint
import xml.etree.ElementTree as ET
from PySide6.QtCore import QPointF
from PySide6.QtGui import QPolygonF
import numpy as np
def read_xml(filename: str, robot: Robot_class, obstacles: ObstacleManager, goal_point: GoalPoint):
    """Чтение xml файла filename"""
    robot.reset()
    obstacles.reset()
    goal_point.reset()

    tree = ET.parse(filename)
    
    root = tree.getroot()
    robot_info = root.find('robot_info')
    joints = robot_info.find('joints')
    start_configuration = root.find('start_configuration')
    for joint,joint_info, st_conf in zip(robot,joints,start_configuration):
        joint.update_left_limit(float(joint_info.find('limit1').text))
        joint.update_start_angle(float(st_conf.text))
        joint.update_right_limit(float(joint_info.find('limit2').text))
        joint.update_length(float(joint_info.find('length').text)*100)
        joint.rotate(float(st_conf.text))
        
        robot.add_joint()
        
    robot.pop_joint()
                        
    goal_info = root.find('goal_point')
    
    goal_point.set_angle((float(goal_info.find('angle1').text)-180)*np.pi/180)
    goal_point.create_dot(QPointF(float(goal_info.find('x').text)*100,float(goal_info.find('y').text)*100))
    
    
    scene_info = root.find('scene')
    
    for polygon in scene_info.findall('polygon'):
        obstacles.add_obstacle()
        poli = obstacles.obstacles[-1]
        poli.set_dots_visible(False)
        for vertex in polygon.findall('vertex'):
            poli.add_dot(QPointF(float(vertex.find('x').text)*100,float(vertex.find('y').text)*100))
        poli.set_dots_visible(False)
    

"""<input_info>
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
</input_info>"""
