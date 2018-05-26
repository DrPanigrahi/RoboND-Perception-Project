#!/usr/bin/env python

# Author(s): Smruti Panigrahi

# Import modules
import rospy
import pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2

import pcl_helper as pclh
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint




# Statistical outlier filter
def statistical_filter_example():
    '''
    -*- coding: utf-8 -*-
    port of
    http://pointclouds.org/documentation/tutorials/statistical_outlier.php
    you need to download
    http://svn.pointclouds.org/data/tutorials/table_scene_lms400.pcd
    '''
    p = pcl.load("table_scene_lms400.pcd")

    fil = p.make_statistical_outlier_filter()

    fil.set_mean_k(50)
    fil.set_std_dev_mul_thresh(1.0)
    pcl.save(fil.filter(), "table_scene_lms400_inliers.pcd")

    fil.set_negative(True)
    pcl.save(fil.filter(), "table_scene_lms400_outliers.pcd")


# Statistical outlier filter
def statistical_outlier_filter(cloud, mean_k=50, std_dev_mul_thresh=0.1):
    # Create a Statistical Outlier Filter object to remove unwanted noise in point cloud
    sof = cloud.make_statistical_outlier_filter()

    sof.set_mean_k(mean_k)
    sof.set_std_dev_mul_thresh(std_dev_mul_thresh)
    
    filtered_inliers = sof.filter()
    #filename1 = 'table_filtered_statistical_inliers.pcd'
    #pcl.save(filtered_inliers, filename1)
    
    sof.set_negative(True)
    filtered_outliers = sof.filter()
    #filename2 = 'table_filtered_statistical_outliers.pcd'
    #pcl.save(filtered_outliers, filename2)

    return filtered_inliers, filtered_outliers


# Voxel-grid down-sampling filter
def voxel_downsampling(cloud, LEAF_SIZE=0.005):
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud.make_voxel_grid_filter()
    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
    #filename = 'voxel_downsampled.pcd'
    #pcl.save(cloud_filtered, filename)
    return cloud_filtered


# Pass-through filter
def pass_through_filter(cloud_vox_filtered, axis_min=0.77, axis_max=1.1, filter_axis='z'):
    # Create a PassThrough filter object to create a bounding box
    cloud_psf = cloud_vox_filtered.make_passthrough_filter()
    cloud_psf.set_filter_field_name(filter_axis)
    cloud_psf.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = cloud_psf.filter()
    #filename = 'cloud_psf.pcd'
    #pcl.save(cloud_filtered, filename)
    return cloud_filtered


# RANSAC plane segmentation
def ransac(cloud_psf, max_distance=0.01):
    # Create the segmentation object
    seg = cloud_psf.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
  
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers
    extracted_inliers = cloud_psf.extract(inliers, negative=False)
    #filename = 'extracted_inliers.pcd'
    #pcl.save(extracted_inliers, filename)   # Save pcd for table

    # Extract outliers
    extracted_outliers = cloud_psf.extract(inliers, negative=True)
    #filename = 'extracted_outliers.pcd'
    #pcl.save(extracted_outliers, filename)  # Save pcd for tabletop objects

    return extracted_inliers, extracted_outliers


# Euclidean clustering algorithm to segment the RANSAC inlier points into individual objects.
def euclidean_clustering(cloud):
    # Convert XYZRGB point cloud to XYZ since EC uses spatial info only
    cloud_xyz = pclh.XYZRGB_to_XYZ(cloud)
    # Use k-d tree to decrease the computational burden of searching for neighboring points
    tree = cloud_xyz.make_kdtree()
    # Create a cluster extraction object
    ec = cloud_xyz.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(10000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    return cluster_indices


def visualize_clusters(cloud, cluster_indices):
    #cloud_xyz = pclh.XYZRGB_to_XYZ(cloud)
    #Assign a color corresponding to each segmented object in scene
    cluster_color = pclh.get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        color = pclh.rgb_to_float(cluster_color[j])
        for idx in indices:
            p = cloud[idx]
            color_cluster_point_list.append([p[0], p[1], p[2], color])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    return cluster_cloud



def main(): 
    # Run the following on command line inside the folder where the .pcd file resides
    # pcl_viewer tabletop.pcd

    # Load Point Cloud file
    cloud = pcl.load('tabletop.pcd')

    stat_inliers, stat_outliers = statistical_outlier_filter(cloud)
    clFil = voxel_downsampling(stat_inliers)
    psFil = pass_through_filter(clFil)
    RANSAC_inliers, RANSAC_outliers = ransac(psFil)
    #statistical_filter_example()


if __name__ == "__main__":
    main()