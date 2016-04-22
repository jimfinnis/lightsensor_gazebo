# lightsensor_gazebo
A trivial "light sensor" for Gazebo, which traverses the world calculating angles to specially marked models.

Lacking an omnidirectional camera sim, and realising any sim would be far too slow for my purposes, I realised my application just
needs to head toward or away from different coloured lights.

This plugin provides that functionality. It traverses the models in the world, looking for those with names of the form "lightrgbXXX"
where XXX is a hexadecimal RGB colour. The relative angle and distance of the "light" is taken into account, and a buffer of RGB pixels -
a virtual omnidirectional planar camera - is built up. This is then broadcast on a ROS topic.
