# Структура входного XML файла:
```
<?xml version='1.0' encoding='utf-8'?>
<input_info>
  <robot_info>
    <joints>
      <joint number="0" length="1.0" width="0.2" limit_min="-180" limit_max="180" />
      <joint number="1" length="1.0" width="0.2" limit_min="-180" limit_max="180" />
      <joint number="2" length="1.0" width="0.2" limit_min="-180" limit_max="180" />
      <joint number="3" length="1.0" width="0.2" limit_min="-180" limit_max="180" />
    </joints>
  </robot_info>
  <start_configuration>
    <angle number="0">99.78240703180728</angle>
    <angle number="1">-10.004430967244275</angle>
    <angle number="2">5.4805837620885995</angle>
    <angle number="3">-6.746088380500396</angle>
  </start_configuration>
  <goal_point x="-3.152173913043478" y="0.21739130434782608" angle="118.07248693585294" angle_tolerance="10" />
  <scene>
    <polygon>
      <vertex x="-2.5652173913043477" y="0.6304347826086957" />
      <vertex x="-2.58695652173913" y="0.08695652173913043" />
      <vertex x="-1.826086956521739" y="0.15217391304347824" />
      <vertex x="-1.9260869565217391" y="0.5956521739130435" />
    </polygon>
    <polygon>
      <vertex x="1.6173913043478259" y="3.052173913043478" />
      <vertex x="1.7043478260869565" y="2.378260869565217" />
      <vertex x="2.4" y="2.4" />
      <vertex x="2.269565217391304" y="3.139130434782609" />
    </polygon>
    <polygon>
      <vertex x="1.8565217391304347" y="1.5521739130434782" />
      <vertex x="1.7478260869565216" y="0.7913043478260868" />
      <vertex x="2.660869565217391" y="0.8565217391304347" />
      <vertex x="2.6173913043478256" y="1.5739130434782607" />
    </polygon>
  </scene>
<csv>91.123303,-2.996995,-1.925675,3.105764
97.123303,-8.996995,4.074325,9.105764
97.123303,-8.996995,4.074325,15.105764
.......................................
199.123303,117.003005,88.074325,147.105764
</csv></input_info>
</input_info>
```
