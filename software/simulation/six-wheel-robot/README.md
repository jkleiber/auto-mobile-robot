
# Six Wheel Robot

This contains the gazebo models and plugins for a six wheel robot.

## Using Model in Gazebo

1. Open Gazebo
2. Go to the Insert Tab
3. Add Path
4. Add the models directory to the Gazebo Path
5. Now the robot can be used in all its configurations in gazebo!

## Using Worlds in Gazebo
There are example worlds in the worlds directory. You can run a particular world by commanding
```
gazebo <world-file>.sdf
```

## Tips
Run 
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(pwd)
```
when compiling a plugin from within its build directory