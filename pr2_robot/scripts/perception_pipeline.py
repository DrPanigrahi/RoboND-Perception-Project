#!/usr/bin/env python

# Author(s): Smruti Panigrahi

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
import features as ft
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
#from sensor_stick.pcl_helper import *
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

import pcl_helper as pclh
import filtering as flt

from geometry_msgs.msg import Pose



# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


# Integer to ros int32 object
def make_ros_int32(py_integer):
    ros_int32 = Int32()
    ros_int32.data = py_integer
    return ros_int32

# Python string to ros string.
def make_ros_string(py_string):
    ros_string = String()
    ros_string.data = py_string
    return ros_string

# ros pose from position coordinates
def make_ros_pose(x, y, z):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    return pose

# Yaml dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Find detected object with label (return none if it was not found)
def match_object_label(detected_objects_list, yaml_label):
    for detected_object in detected_objects_list:
        if detected_object.label == yaml_label:
            return detected_object
    return None

# Calculate position of centroid of the objects and create ros pick pose
def calc_object_size(item):
    points_array = pclh.ros_to_pcl(item.cloud).to_array()
    centroid = np.mean(points_array, axis=0)[:3]
    max_dim = np.max(points_array, axis=0)[:3]
    min_dim = np.min(points_array, axis=0)[:3]
    object_size = max_dim - min_dim
    length, width, height = make_ros_pose(object_size[0], object_size[1], object_size[3])
    print("The dimensions of the detected item are: length=%f, width=%f, height=%f" % (length. width, height))
    return length, width, height, centroid

# Calculate position of centroid of the objects and create ros pick pose
def create_pick_pose(item):
    points_array = pclh.ros_to_pcl(item.cloud).to_array()
    centroid = np.mean(points_array, axis=0)[:3]
    return make_ros_pose(centroid[0], centroid[1], centroid[2])

# Create ros place pose depending on the item group 'red' or 'green'
def create_place_pose(item_group, total_items_picked, clearance=0.1):
    pz = 0.605 
    # Place item-group 'red' in the left bin and 'green' in the 'right' bin
    if item_group == 'red':
        py = 0.7
    else:
        py = -0.7
    # Place items side-by-side instead of stacking them at one location
    px = -0.1 - float(total_items_picked)*clearance

    return make_ros_pose(px,py,pz)

# Select the pr2 arm name ('left'/'right') depending on the item group 'red'/'green'
def select_arm(item_group):
    arm_name = 'left' if item_group == 'red' else 'right'
    return make_ros_string(arm_name)

# Send command to robot arm to turn
def turn_pr2(start_angle, end_angle):
    joint_controller_pub.publish(end_angle)
    rospy.sleep(abs(end_angle - start_angle) / np.pi * 36.0)


# Callback function for your Point Cloud Subscriber
def pcl_callback(ros_pcl_msg):

# Exercise-1 TODOs:
    # TODO: Load point cloud data
    # TODO: Statistical Outlier Filtering
    # TODO: Voxel Grid Downsampling
    # TODO: PassThrough Filter
    # TODO: RANSAC Plane Segmentation
    # TODO: Extract inliers and outliers
    # TODO: Save the inliers in .pcd file
    # TODO: Run the inliers.pcd file on terminal
# Exercise-2 TODOs:
    # TODO: Convert ROS msg to PCL data   
    # TODO: Statistical Outlier Filtering
    # TODO: Voxel Grid Downsampling
    # TODO: PassThrough Filter
    # TODO: RANSAC Plane Segmentation
    # TODO: Extract inliers and outliers
    # TODO: Euclidean Clustering
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # TODO: Convert PCL data to ROS messages
    # TODO: Publish ROS messages
# Exercise-3 TODOs: 
    # TODO: Classify the clusters! (loop through each detected cluster one at a time)
        # TODO: Grab the points for the cluster
        # TODO: Compute the associated feature vector
        # TODO: Make the prediction
        # TODO: Publish a label into RViz
        # TODO: Add the detected object to the list of detected objects.
    # TODO: Publish the list of detected objects


    # TODO: Convert ROS msg of type PointCloud2 to PCL data PointXYZRGB format
    pcl_data = pclh.ros_to_pcl(ros_pcl_msg)

    # TODO: Statistical filter
    stat_inlier, stat_outlier = flt.statistical_outlier_filter(pcl_data)
    
    # TODO: Voxel Grid Downsampling
    vox_filt = flt.voxel_downsampling(stat_inlier)

    # TODO: PassThrough Filter
    pass_filt_z = flt.pass_through_filter(vox_filt, 0.6, 1.2, 'z')
    pass_filt_zy = flt.pass_through_filter(pass_filt_z, -0.5, 0.5, 'y')
    pass_filt_zyx = flt.pass_through_filter(pass_filt_zy, 0.4, 1.2, 'x')
    
    # TODO: RANSAC Plane Segmentation
    # TODO: Extract inliers (objects) and outliers (table)
    cloud_table, cloud_objects = flt.ransac(pass_filt_zyx)
    
    # TODO: Euclidean Clustering
    cluster_indices = flt.euclidean_clustering(cloud_objects)

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_cloud = flt.visualize_clusters(cloud_objects, cluster_indices)

    # TODO: Convert PCL data to ROS messages
    ros_table_cloud = pclh.pcl_to_ros(cloud_table)
    ros_objects_cloud = pclh.pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pclh.pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_table_pub.publish(ros_table_cloud)
    pcl_objects_pub.publish(ros_objects_cloud)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    
    # TODO: Classify the clusters and create list of detected objects    
    detected_objects, detected_objects_labels = detect_objects(cloud_objects, cluster_indices)   

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    detected_objects_pub.publish(detected_objects)
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

def detect_objects(cloud_objects, cluster_indices):

    # TODO: Classify the clusters!
    detected_objects_labels = []
    detected_objects = []
    labeled_features = []

    for index, pts_list in enumerate(cluster_indices):
        # TODO: Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: Convert the cluster from pcl to ROS using helper function
        ros_cluster = pclh.pcl_to_ros(pcl_cluster)

        # TODO: Extract histogram features
        hists_color = ft.compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        hists_normals = ft.compute_normal_histograms(normals)
        feature = np.concatenate((hists_color, hists_normals))
        #labeled_features.append([feature, model_name])

        # TODO: Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        # TODO: Retrieve the label for the result
        label = encoder.inverse_transform(prediction)[0]
        # TODO: Add it to detected_objects_labels list
        detected_objects_labels.append(label)

        # TODO: Publish a label into RViz
        white_cloud = pclh.XYZRGB_to_XYZ(cloud_objects)
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .2
        object_markers_pub.publish(make_label(label,label_pos, index))

        # TODO: Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    return detected_objects, detected_objects_labels


# function to load parameters and request PickPlace service
def pr2_mover(detected_objects):

    # TODO: Initialize variables and get/read parameters
    # TODO: Rotate PR2 in place to capture side tables for the collision map
    # TODO: Loop through the pick list
        # TODO: Parse parameters into individual variables
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        # TODO: Create 'place_pose' for the object
        # TODO: Assign the arm to be used for pick_place
        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

    # TODO: Initialize variables
    dict_list = []
    total_items_matched = 0
    total_items_picked = 0
    # TODO: Get/read the object_list from the pick_list_xx.yaml file in config folder
    object_list_param = rospy.get_param('/object_list')
    # TODO: Get/read the test scene number from the pick_place_project.launch file in launch folder
    test_scene_num_int = rospy.get_param('/test_scene_num')
    test_scene_num = make_ros_int32(test_scene_num_int)

   # TODO: Rotate PR2 in place to capture side tables for the collision map
    turn_pr2(0.0, -np.pi/2)
    turn_pr2(-np.pi/2, np.pi/2)
    turn_pr2(np.pi/2, 0.0)

    # TODO: Loop through the pick list obtained from the yaml file
    for object_dict in object_list_param:
        # TODO: Parse parameters into individual variables
        item_name = object_dict['name']
        item_group = object_dict['group']
        print("Test scene %d: requesting to pick (%s) and place in (%s) bin." % (test_scene_num_int, item_name, item_group))

        # TODO: Match the item from yaml file to the detected_objects_list and get the PointCloud 
        item = match_object_label(detected_objects, item_name)
        if  item == None:
            continue
        total_items_matched += 1
        print "Test scene %d: total number of items matched is %d." % (test_scene_num_int, total_items_matched)
        
        # TODO: Get the item nale/label to be picked
        item_name = make_ros_string(item_name)
        # TODO: For a given object obtain its centroid and create 'pick pose'
        pick_pose = create_pick_pose(item)
        # TODO: Create 'place_pose' for the object
        place_pose = create_place_pose(item_group, total_items_picked)
        # TODO: Assign the arm to be used for pick_place
        arm_name = select_arm(item_group)
        # TODO: Create a list of dictionaries for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, item_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        
        # TODO: Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            # TODO: Insert message variables to be sent as a service request
            response = pick_place_routine(test_scene_num, item_name, arm_name, pick_pose, place_pose)
            if response.success:
                total_items_picked += 1
            print ("Response to pick-place service: ",response.success)
            rospy.loginfo('"Test scene {}: picking {} with {} arm and placing in {} bin'.format(test_scene_num, item_name, arm_name, item_group))
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    # TODO: Output your request parameters into output yaml file
    yaml_filename = "Output_%d.yaml" % test_scene_num_int
    print("Writing to %s" % yaml_filename)
    send_to_yaml(yaml_filename, dict_list)
    print("Test scene %d: %d items picked out of %d" % (test_scene_num_int, total_items_picked, total_items_matched))


if __name__ == '__main__':
    
    # TODO: ROS node initialization
    # TODO: Create Subscribers
    # TODO: Create Publishers
    # TODO: Load Model From disk
    # TODO: Initialize color_list
    # TODO: Spin while node is not shutdown

    
    # TODO: ROS node initialization
    rospy.init_node('object_recognition', anonymous = True)

    # TODO: Create Publishers to visualize PointCloud topics in rviz
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size = 1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size = 1)
    joint_controller_pub = rospy.Publisher("/pr2/world_joint_controller/command", Float64, queue_size=1)
    
    # TODO: Create Subscribers
    #pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size = 1)
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size = 1)
    
    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # TODO: Initialize color_list
    pclh.get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

