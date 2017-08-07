### Project: Perception Pick & Place

This project is on 3D perception. It uses a advanced robotics platform called the PR2 in simulation together with RBG-D data.

![pr2 and table](./images/scenario.png)


#### Perception Exercise 1, 2 and 3

This project is built on top of the Perception exercises provided by Udacity RoboND. The link to the completed exercise. It also contains a writeup that explains all the techniques explored.

https://github.com/lisaljl/Udacity-RoboND-Perception
https://github.com/lisaljl/Udacity-RoboND-Perception/blob/master/writeup.md

#### Perception project

There are 3 different scenarios (tabletop setups), where the number and type of objects differs for the different scenarios. Below are the results:

##### Test1 world and pick list 1

[Output yaml](https://github.com/lisaljl/Udacity-RoboND-Perception-Project/blob/master/pr2_robot/output/output_1.yaml)

![pr2 and table world 1](./images/pick_list_1.png)

##### Test2 world and pick list 2

[Output yaml](https://github.com/lisaljl/Udacity-RoboND-Perception-Project/blob/master/pr2_robot/output/output_2.yaml)

![pr2 and table world 2](./images/pick_list_2.png)

##### Test3 world and pick list 3

[Output yaml](https://github.com/lisaljl/Udacity-RoboND-Perception-Project/blob/master/pr2_robot/output/output_3.yaml)

![pr2 and table world 3](./images/pick_list_3.png)


Here are the steps required to detect objects from surroundings:

1. Get input from ROS
2. ros to pcl
3. Voxel grid downsampling
4. PassThrough Filter
5. RANSAC Plane Segmentation
6. Extract inliers and outliers
7. Euclidean Clustering
8. Create Cluster-Mask Point Cloud to visualize each cluster separately
9. Classify the clusters! (loop through each detected cluster one at a time)
10. Grab the points for the cluster
11. Compute the associated feature vector
12. Make the prediction
13. Publish a label into RViz
14. Add the detected object to the list of detected objects.




Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  


