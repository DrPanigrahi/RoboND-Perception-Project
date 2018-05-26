#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import pcl_helper as pclh
import filtering as flt


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(ros_pcl_msg):
# Exercise-1 TODOs:
    # TODO: Load point cloud data
    # TODO: Voxel Grid Downsampling
    # TODO: PassThrough Filter
    # TODO: RANSAC Plane Segmentation
    # TODO: Extract inliers and outliers
    # TODO: Save the inliers in .pcd file
    # TODO: Run the inliers.pcd file on terminal
# Exercise-2 TODOs:
    # TODO: Convert ROS msg to PCL data
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
    pcl_data = stat_inlier
    
    # TODO: Voxel Grid Downsampling
    vox_filt = flt.voxel_downsampling(pcl_data)

    # TODO: PassThrough Filter
    pass_filt = flt.pass_through_filter(vox_filt)
    
    # TODO: RANSAC Plane Segmentation
    # TODO: Extract inliers (objects) and outliers (table)
    cloud_table, cloud_objects = flt.ransac(pass_filt)
    
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


    # TODO: Classify the clusters!
    detected_objects_labels = []
    detected_objects = []
    labeled_features = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster = pclh.pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        # Extract histogram features
        hists_color = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        hists_normals = compute_normal_histograms(normals)
        feature = np.concatenate((hists_color, hists_normals))
        #labeled_features.append([feature, model_name])


        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        white_cloud = pclh.XYZRGB_to_XYZ(cloud_objects)
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)


if __name__ == '__main__':

    # TODO: ROS node initialization
    # TODO: Create Subscribers
    # TODO: Create Publishers
    # TODO: Load Model From disk
    # TODO: Initialize color_list
    # TODO: Spin while node is not shutdown


    # TODO: ROS node initialization
    rospy.init_node('object_segmentation', anonymous = True)

    # TODO: Create Publishers to visualize PointCloud topics in rviz
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size = 1)
    
    # Initialize color_list
    pclh.get_color_list.color_list = []

    # Create Publishers
    # TODO: here you need to create two publishers
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size = 1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size = 1)
    
    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
