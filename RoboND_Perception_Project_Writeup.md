[//]: # (Image References)
[image1]: ./pr2_robot/output/PR2_Robot_Front.jpg
[image2]: ./pr2_robot/output/PR2_Robot_Back.jpg
[image3]: ./pr2_robot/output/RViz_Sensor_Stick.png
[image4]: ./pr2_robot/output/test3-world.png
[image5]: ./pr2_robot/output/feature-capture.png
[image6]: ./pr2_robot/output/exercise3_detection.png
[image7]: ./pr2_robot/output/exercise3_clustering.png
[image8]: ./pr2_robot/output/exercise3_clustering-2.png
[image9]: ./pr2_robot/output/norm_conf_mat5_100_set2.png
[image10]: ./pr2_robot/output/norm_conf_mat8_100_set3.png
[image11]: ./pr2_robot/output/pcl-table-set1.png
[image12]: ./pr2_robot/output/pcl-table-set2.png
[image13]: ./pr2_robot/output/pcl-table-set3.png
[image14]: ./pr2_robot/output/pcl-objects-set1.png
[image15]: ./pr2_robot/output/pcl-objects-set2.png
[image16]: ./pr2_robot/output/pcl-objects-set3.png
[image17]: ./pr2_robot/output/pcl-cluster-set1.png
[image18]: ./pr2_robot/output/pcl-cluster-set2.png
[image19]: ./pr2_robot/output/pcl-cluster-set3.png
[image20]: ./pr2_robot/output/picking-item.png
[image21]: ./pr2_robot/output/pick-list2-using-model3-detection.png
[image22]: ./pr2_robot/output/pick-list2-using-model2-detection.png
[image23]: ./pr2_robot/output/pick-list2-using-model2-matching.png
[image24]: ./pr2_robot/output/pick-list2-items-picked.png

[video01]: ./pr2_robot/output/Perception_Project_Video.mov


# Project: Perception Pick & Place
In this project we will be performing object detection and recognition using various computervision and filtering techniques. We will then bring the perception pipeline onto a gazebo environment with a PR2 Robot to pick and sort the objects into designated bins (a part of the Amazon Robotics challenge). The PR2 robot has two arms and a hip joint which allows it to rotate in place. The PR2 is also equiped with a RGBD/Stereo camera on the top to be able to see objects of interest and sort them accordingly. Our goal in this project is to work with the PointCloud data obtained from the PR2's RGBD camera and process them to perform various filtering, clustering and segmentation tasks in order to recognise the object. Once the objects has been recognised we then need to provide the PR2 robot with the positions and orientations of these objects for the Robot to pick and place the items in desired bins. 
![alt text][image1] ![alt text][image2]

---
**The goals / steps of this project are the following:**
1. Extract features and train an SVM model from objects in `pick_list_*.yaml` in `/pr2_robot/config/` directory for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.

**Extra Challenges: Complete the Pick & Place:**
9. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
10. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
11. Rotate the robot back to its original state.
12. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
13. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
14. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
15. For a bigger challenge, load up the `challenge.world` scenario and apply your perception pipeline!

# Generating Features
To get started generating features, launch the training.launch file to bring up the Gazebo environment. An empty environment should appear with only the sensor stick robot in the scene. Assuming we already have a Catkin workspace setup on your virtual machine (or locally) the first step is to copy or move the /sensor_stick directory and all of its contents to ~/catkin_ws/src.
1. clone the git repo RoboND-Perception-Exercise 
2. copy sensor_stick folder inside Exercise-3 to the catkin_ws folder. 
3. delete the sensor_stick folders inside Exercise-2 and Exercise-3 folder. 
```sh
$ cp -r ~/RoboND-Perception-Exercises/Exercise-2/sensor_stick ~/catkin_ws/src/
$ cd ~/catkin_ws/src/RoboND-Perception-Exercises/Exercise-2
$ rm -rf sensor_stick
$ cd ~/catkin_ws/src/RoboND-Perception-Exercises/Exercise-3
$ rm -rf sensor_stick
```
Next, use rosdep to grab all the dependencies you need to run this exercise.
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
After that, run catkin_make to build the dependencies.
```
$ catkin_make
```
Add the following lines to your .bashrc file
```sh
export GAZEBO_MODEL_PATH=~/catkin_ws/src/sensor_stick/models
source ~/catkin_ws/devel/setup.bash
```
Now, we should be all setup to launch the environment! Run the following command to launch the scene in Gazebo and RViz:

$ roslaunch sensor_stick robot_spawn.launch
Your RViz window should look like this:
![alt text][image3] 

The scene in the lower left is the view as seen from an RGB-D camera mounted on top of the blue stick robot we see in the scene, hence the name "sensor_stick" for the exercise. If you don't see the exact image above and see an error in the RViz when you launch the environment, you can change the "Style" of your PointCloud2 data by clicking on it and changing it from "Points" to "Flat Squares". Next, we will write a ROS node to publish the point cloud as seen from the camera on the sensor stick!

**Filtering**
Once we have our camera data, start out by applying various filters. To remove the noise, the first filter we should apply is the statistical outlier filter.

Note, the statistical outlier filter in python-pcl might be broken, so if you're getting an error when running the statistical outlier filter that looks like this:

```sh
Error: TypeError: __cinit__() takes exactly 1 positional argument (0 given)
```
you need to re-install python-pcl:
```sh
$ cd ~/RoboND-Perception-Exercises/python-pcl
$ python setup.py build
$ sudo python setup.py install
```
**Table Segmentation**
Next, perform RANSAC plane fitting to segment the table from the objects on top in the scene. 

**Clustering**
Use the Euclidean Clustering technique to separate the objects into distinct clusters, thus completing the segmentation process.

# Object Recognition
For this project, we have a variety of different objects to identify. Essentially, there are three different worlds or scenarios that we are going to work with where each scenario has different items on the table in front of the robot. These worlds are located in the /pr2_robot/worlds/ folder, namely the test_*.world files.

By default, we start with the test1.world but we can modify that in the pick_place_project.launch file in the /pr2_robot/launch/ folder:
```sh
  <!--Launch a gazebo world-->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <!--TODO:Change the world name to load different tabletop setup-->
    <arg name="world_name" value="$(find pr2_robot)/worlds/test1.world"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="paused" value="false"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
```

![alt text][image4] 

Generate a training set of features for the objects in the pick lists (see the pick_list_*.yaml files in /pr2_robot/config/). Each pick list corresponds to a world and thus indicates what items will be present in that scenario. To generate the training set, we will have to modify the models list in the capture_features.py script.

```python
if __name__ == '__main__':
  rospy.init_node('capture_node')

  # Modify following list with items from pick_list_*.yaml
      # From pick_list_3.yaml
    models = [
        'biscuits',
        'soap',
        'soap2',
        'book',
        'glue',
        'sticky_notes',
        'snacks',
        'eraser']
```
From Perception Exercise, we detected and clustered all objects accurately as shown beloow:
![alt text][image5] 
![alt text][image6] 
![alt text][image7]

We choose number of training example for each object by changing the for loop as below:
```python
for model_name in models:
        spawn_model(model_name)

        for i in range(100):
            # make five attempts to get a valid point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()
```
Here I have choosen 100 training examples for each object. 

We then run the capture_features.py as to capture the color and surface/edge features. 
```sh
$ cd ~/catkin_ws
$ catkin_make
$ roslaunch pr2_robot training.launch
```

Open a new terminal and run the capture_features.py. This will take quite a long time, about 1 seconds per spawn, taking about 2-3 minutes per object. Hence it might take about 20 minutes to capture the features for the pick_list_3 items.
```sh
$ rosrun pr2_robot capture_features.py
```

We must train a model to recognise our objects. Since we already have objects with labels we can train a supervised machine learning classifer to train a model and then use the trained model to predict the objects in the test scenario.  In our case we will use a type of supervised model called support-vector machine (SVM). Once we have captured all the features, we then go ahead and train our SVM model with a linear cornel by running the train_svm.py. We can also train the model with different kernel such as 'rbf' and 'poly'. 
```sh
$ rosrun pr2_robot train_svm.py
```
We will see the accuracy of our tranined model from the normalized confusion matrix using the cross-validation set. 

The SVM I used was trained by sampling 100 randomly generated poses for each item. The confusion matrix of the cross-validation set using the trained classifier model for pick-list-2 (by capturing the features for 5 objects) is shown below. The classification accuracy was between 88% for buiscuits/book to 95% for soap.
![alt text][image9] 

The confusion matrix of the cross-validation set using the trained classifier model for pick-list-3 (by capturing the features for 8 objects) is shown below. The classification accuracy was between 80% for snacks to 93% for eraser.
![alt text][image10] 

To test with the project, we first choose the pick list by changing the test scene number in line-3 of the pick_place_project.launch file (here we choose the objects from pick_list_2.yaml file containing 5 items).
```sh
<arg name="test_scene_num" value="2"/>
```
then we launch the project by running:
```sh
$ roslaunch pr2_robot pick_place_project.launch
```
and then we run our perception_pipeline code for object recognition and pick-place operation.
```sh
$ rosrun pr2_robot perception_pipeline.py
```
We arrive at results similar to what we got in Exercise-3 but this time for new objects in a new environment! Keep an eye out for errors in the terminal and if Gazebo/RViz crashes or doesn't come up just give it another try, sometimes takes a few attempts!

The object detection, clusering and recognition of pick_list_1 is shown below:
![alt text][image11] 
![alt text][image14] 
![alt text][image17] 

The object detection, clusering and recognition of pick_list_2 is shown below:
![alt text][image12] 
![alt text][image15] 
![alt text][image18] 

The object detection, clusering and recognition of pick_list_3 is shown below:
![alt text][image13] 
![alt text][image16] 
![alt text][image19] 

Using the model that was trained with all 8 items, I was able to identify all 3 items correctly from pick-list-1(100% accuracy), one mis-classification in pick-list-2 (80% accuracy) and two mis-classifications in pick-list-3 (75% accuracy).
![alt text][image21] 

However, training the model with items from the pick-list-2 only, lead to 100% accuracy in object recognition for pick-list-2. 
![alt text][image22] 

The matched items can be seen in the terminal
![alt text][image23] 

Once the perception task is over then the Robot perfroms the pick and place operation. We extract the item group from the .yaml pick-list files and feed them into the robot to place the objects in the right bin. Once the task is complete the output.yaml file is generated and saved in the output folder. The figure below shows the PR2 in active mode for picking and placing the items.
![alt text][image24]

Overall there were a lot of computer vision stuff that I learned by doing the exercises and applying them into the project. I was however, quite unhappy with the Robot's performance. Majority of the time the Robot could not grasp the objects tightely, so the objects always dropped off the gripper and fly arround. Learned lot of techniques but in the end it was a sad robot! It would be good to investigate further on the Robot arm motion, as it goes crazy and swings around eratically while trying to pick and place the objects. I might play with that at a later time. 


