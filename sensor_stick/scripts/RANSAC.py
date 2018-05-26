# Import PCL module
import pcl


# Voxel Grid filter
def voxFilter(cloud):
	# Create a VoxelGrid filter object for our input point cloud
	vox = cloud.make_voxel_grid_filter()

	# Choose a voxel (also known as leaf) size
	# Note: this (1) is a poor choice of leaf size   
	# Experiment and find the appropriate size!
	LEAF_SIZE = 0.01   

	# Set the voxel (or leaf) size  
	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

	# Call the filter function to obtain the resultant downsampled point cloud
	cloud_filtered = vox.filter()
	filename = 'voxel_downsampled.pcd'
	pcl.save(cloud_filtered, filename)

	return cloud_filtered


# PassThrough filter
def passThroughFilter(clFil):
	cloud_filtered = clFil
	# Create a PassThrough filter object.
	passthrough = cloud_filtered.make_passthrough_filter()

	# Assign axis and range to the passthrough filter object.
	filter_axis = 'z'
	passthrough.set_filter_field_name(filter_axis)
	axis_min = 0.7
	axis_max = 1.1
	passthrough.set_filter_limits(axis_min, axis_max)

	# Finally use the filter function to obtain the resultant point cloud. 
	cloud_filtered = passthrough.filter()
	filename = 'pass_through_filtered.pcd'
	pcl.save(cloud_filtered, filename)
	return cloud_filtered


# RANSAC plane segmentation
def RANSAC(psFil):
	cloud_filtered = psFil
	# Create the segmentation object
	seg = cloud_filtered.make_segmenter()

	# Set the model you wish to fit
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)

	# Max distance for a point to be considered fitting the model
	# Experiment with different values for max_distance 
	# for segmenting the table
	max_distance = 0.01
	seg.set_distance_threshold(max_distance)

	# Call the segment function to obtain set of inlier indices and model coefficients
	inliers, coefficients = seg.segment()

	# Extract inliers
	extracted_inliers = cloud_filtered.extract(inliers, negative=False)
	filename = 'extracted_inliers.pcd'
	pcl.save(extracted_inliers, filename)	# Save pcd for table

	# Extract outliers
	extracted_outliers = cloud_filtered.extract(inliers, negative=True)
	filename = 'extracted_outliers.pcd'
	pcl.save(extracted_outliers, filename) 	# Save pcd for tabletop objects

	return extracted_inliers, extracted_outliers


def statistical_outlier_filter():
	ptcloud = pcl.load('tabletop.pcd')
	# Much like the previous filters, we start by creating a filter object: 
	statistical_filter = ptcloud.make_statistical_outlier_filter()

	# Set the number of neighboring points to analyze for any given point
	statistical_filter.set_mean_k(50)

	# Set threshold scale factor
	x = 1.0
	# Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
	statistical_filter.set_std_dev_mul_thresh(x)

	# Finally call the filter function for magic
	filtered_inliers = statistical_filter.filter()
	filename1 = 'table_filtered_statistical_inliers.pcd'
	pcl.save(filtered_inliers, filename1)

	statistical_filter.set_negative(True)
	filtered_outliers = statistical_filter.filter()
	filename2 = 'table_filtered_statistical_outliers.pcd'
	pcl.save(filtered_outliers, filename2)


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


def main():	
	# Run the following on command line inside the folder where the .pcd file resides
	# pcl_viewer tabletop.pcd

	# Load Point Cloud file
	cloud = pcl.load_XYZRGB('tabletop.pcd')

	clFil = voxFilter(cloud)
	psFil = passThroughFilter(clFil)
	RANSAC_inliers, RANSAC_outliers = RANSAC(psFil)
	statistical_outlier_filter()
	statistical_filter_example()


if __name__ == "__main__":
	main()