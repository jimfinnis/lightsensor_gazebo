# lightsensor_gazebo

A trivial "light sensor" for Gazebo, which traverses the world
calculating angles to specially marked models and producing
an array of pixel values from the extents of those models.

Lacking an omnidirectional camera sim, and realising any sim would be
far too slow for my purposes, I realised my application just needs to
head toward or away from different coloured lights.

This plugin provides that functionality. It traverses the models in
the world, looking for those with names of the form "lightrgbXXX"
where XXX is a hexadecimal RGB colour. The relative angle and distance
of the "light" is taken into account, and a buffer of RGB pixels - a
virtual omnidirectional planar camera - is built up. This is then
broadcast on a ROS topic.
The simulation is entirely 2D (in the XY plane) and there is no occlusion. If multiple lights
overlap a sensor, they are added together (with each colour channel clamped at 255).

## Output
The output is published on the *pixels* topic by default, is of type LightSensor, which is an array of Pixel, which
is 3 uint8 values (r,g,b). The middle value is the front sensor, with the first half of
the values covering the left side and the second half covering the right side
of the robot.

## watch.py
This script, which requires *pygame*, displays the *pixels* topic as a circle
of coloured squares. It's a useful example of working with the output.

## Elements
* **robotNamespace** - the namespace for the broadcast topic (default none)
* **interval** - the update interval in seconds (default 0.1)
* **bodyName** - the name of the link to which the sensor is attached (required)
* **topic** - the broadcast topic (default *pixels*)
* **pixels** - the resolution of the sensor, as the number of pixels (default 10)

## Example URDF
This attaches a light sensor to a robot which has a link called *base_link*:
```xml
  <gazebo>
    <plugin name="lightsensor" filename="liblightsensor_gazebo.so">
      <link name="base_link"/>
      <interval>0.1</interval>
      <pixels>100</pixels>
    </plugin>
  </gazebo>
```
