#!/usr/bin/env python

# Author(s): Smruti Panigrahi

# Import modules
import rospy
import pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint
#from pcl_helper import *
import pcl_helper as pclh
import filtering as flt


# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(ros_point_cloud_msg):

    # TODO: Convert ROS msg of type PointCloud2 to PCL data PointXYZRGB format
    pcl_data = pclh.ros_to_pcl(ros_point_cloud_msg)

    # TODO: Statistical filter
    stat_inlier, stat_outlier = flt.statistical_outlier_filter(pcl_data)
    
    # TODO: Voxel Grid Downsampling
    vox_filt = flt.voxel_downsampling(stat_inlier)

    # TODO: PassThrough Filter
    pass_filt = flt.pass_through_filter(vox_filt)
    
    # TODO: RANSAC Plane Segmentation
    # TODO: Extract inliers and outliers
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


if __name__ == '__main__':

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

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()