# urdf_visualizer
just put your meshes and urdf file and vizualise it !!

modifications to do to your urdf file:
- change <your_package> to urdf_visualizer (I would like to change it in the future, a hook in xml...)

put your file in urdf/xacro and meshes and launch:

```ros2 launch  urdf_visualizer urdf_vizualiser.launch.py filename:=ridgeback.urdf.xacro```

or 

```ros2 launch  urdf_visualizer urdf_vizualiser.launch.py filename:=ridgeback_structure.urdf```
