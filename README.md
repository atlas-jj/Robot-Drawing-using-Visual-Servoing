# Robot-Drawing-using-Visual-Servoing
###  CMPUT 615 Course Project: A Robotic Photocopy Machine (RPM), University of Alberta


# Overview
+ Human shows the robot a picture, our system can control the robot replicate the same pic in an unrealistic drawing style.
+ chris_tracker:
  + parsing a photo to lines and ellipse.
  + generating strokes and motion plans.
  + subscribing to ar_track_alvar and continously output the error vector.
+ wam_ibvs_visp:
  + receiving strokes and motion plans.
  + controlling using visual servoing, ViSP.
  + Camera configuration: eye-to-hand.

# System Overview
![system](https://github.com/atlas-jj/Robot-Drawing-using-Visual-Servoing/blob/master/media/overview.jpg?raw=true)

# ROS-Graph
![rosgraph](https://github.com/atlas-jj/Robot-Drawing-using-Visual-Servoing/blob/master/media/rosgraph.png?raw=true)

# Results
More results (video clips) can be found in http://home.jinchris.com/RobotDrawing.html

Video example:

```html
https://drive.google.com/file/d/0B9P2CU_oU7BjY0ZUNUdMbnhMY1U/view
```

Drawing a rectangle

![](https://github.com/atlas-jj/Robot-Drawing-using-Visual-Servoing/blob/master/media/a%20rectangle.png?raw=true)

Self-portrait 

![](https://github.com/atlas-jj/Robot-Drawing-using-Visual-Servoing/blob/master/media/selfportrait.jpg?raw=true)


# Reference
J. Jin, "A Robotic Photocopy Machine (RPM)",  https://github.com/atlas-jj/Robot-Drawing-using-Visual-Servoing/blob/master/media/CMPUT_615_Lab_Project_Report_final.pdf
