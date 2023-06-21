"""Чтение XML файла"""


from robot_class import Robot_class
from obstacles import ObstacleManager
from goal_point import GoalPoint
import xml.etree.ElementTree as ET

def read_xml(filename: str, robot: Robot_class, obstacles: ObstacleManager, goal_point: GoalPoint):
    """Чтение xml файла filename"""

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
