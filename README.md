# CyberphysicalSystems

Version control for various files in the F-1-10th codebase that we are editing for our class. Please list instructions for where to scp each file when connected to the car at the top of each file

## How to run the project code

For this you need 7 different ssh windows. In all windows run:

```
#password is 'nvidia'
ssh nvidia@192.168.8.1
cd catkin_ws
```

Now that each window is setup run each command in a different window

```
roscore
```

```
roslaunch src/race/src/basiclanuch.launch
```

```
rosrun zed_wrapper zed_wrapper_node
```

```
rosrun color_tracking vision.py
```

```
rosrun race control.py
```

```
rosrun race kill.py 
#press backspace to change nodes
```

## Reference

* [Final Project requirements](https://www.cs.utexas.edu/~mok/cs378/Spring23/Assignments/FinalProject_2023.htm)
* [Dr. Mok's website](https://www.cs.utexas.edu/~mok/cs378/Spring23/Syllabus.htm)
* [Our Project Proposal](https://docs.google.com/document/d/16QziiXt1vrPoERxozHf6Z90BOtPyxmENEPa5OchgKcY/edit?usp=sharing)
* [Architecture Doc](https://docs.google.com/document/d/1QPOjgJtA0RcuYLephuJJpsuMbt-ten0Xar08ECaYbac/edit?usp=sharing)