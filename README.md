# Next-Best-View Mapping with a Quadrotor

This repo contains an implementation of "Receding Horizon “Next–Best–View” Planner for 3D Exploration" by Bircher et al.

## Dependencies

-   [ROS melodic](https://www.ros.org/)

## Map Creation
To create a map, draw a black and white image in your favorite drawing tool (e.g. pinta) and save it as a png in the config folder. For an example, see test_map.png. Then run map_png_to_txt.py, e.g.
```
python3 map_png_to_txt.py
```
and when it asks for the filename, give the path to the file (either the full path or the path relative to your current directory). This script will convert the image to a grid stored in a text file, where each white pixel is free space, represented by a 0, and each black pixel is an obstacle, represented by a 1. The text file will have the same name as the png file, but instead of .png, the extension will be .txt.
    
## Execution
To launch the planner on a map (created as above), run
```
roslaunch nbv_mapping planner_rviz.launch map:=test_map
```
The above example launches the planner on test_map.txt. To run on a different map, replace the filename.

If you want to change the size of the map cells, change the cube_length parameter in launch/planner_rviz.launch.
