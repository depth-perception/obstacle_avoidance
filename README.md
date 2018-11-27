# obstacle_avoidance

some methods for obstacle avoidance by ros robot

## by point cloud 

this  code is based on  point cloud  from kinect.  A point cloud search box is established in the field of vision. When an object or a pedestrian enters the search box, the center of mass of the point cloud in the search box is calculated. If the center of mass is on the right, the robot turns left. If the center of mass is on the left, the robot turns right

## by ultrasonic

There is 6 ultrasonic sensors in my ros robot, i  just use the three sensors which is in the left front, front, right front. The obstacle avoidance logic description is shown in the following figure.



![è¶å£°æ³¢é¿éç­ç¥.jpeg](http://www.corvin.cn/wp-content/uploads/image/20180330/1522389856422850.jpeg)