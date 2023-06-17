"""Создание XML файла по введённым в сцену объектам"""


from robot_class import Robot_class
from obstacles import ObstacleManager
from goal_point import GoalPoint
import xml.etree.ElementTree as ET

GOAL_POINT_TRESHOLD = 5 #абсолютная погрешность угла конечной точки
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
            
            x = ET.SubElement(vertex_xml,"x")
            x.text = str(vertex.x())
            
            y = ET.SubElement(vertex_xml,"y")
            y.text = str(vertex.y())
            
    
def add_goal_point(root:ET.Element, goal_point:GoalPoint):
    goal_point_xml = ET.SubElement(root,'goal_point')
    x = ET.SubElement(goal_point_xml,"x")
    x.text = str(goal_point.x())
    
    y = ET.SubElement(goal_point_xml,"y")
    y.text = str(goal_point.y())
    
    angle1 = ET.SubElement(goal_point_xml,"angle1")
    angle1.text = str(goal_point.angle() - GOAL_POINT_TRESHOLD)
    
    angle2 = ET.SubElement(goal_point_xml,"angle2")
    angle2.text = str(goal_point.angle() + GOAL_POINT_TRESHOLD)    
    
    

def add_start_conf(root:ET.Element,robot:Robot_class):    
    start_configuration = ET.SubElement(root,'start_configuration')
    for joint in robot:
        angle = ET.SubElement(start_configuration,"angle")
        angle.text = str(joint.start_angle)
                
def add_robot_info(root:ET.Element,robot:Robot_class):
    robot_info = ET.SubElement(root,'robot_info')
    joints = ET.SubElement(robot_info,'joints')
    
    for robot_joint in robot:
        joint = ET.SubElement(joints,'joint')
        length = ET.SubElement(joint,'length')
        length.text = str(robot_joint.length/100)
        width = ET.SubElement(joint,'width')
        width.text = str(robot_joint.width/100)
        limit1 = ET.SubElement(joint,'limit1')
        limit1.text = str(robot_joint.left_angle)
        limit2 = ET.SubElement(joint,'limit2')
        limit2.text = str(robot_joint.right_angle)

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
