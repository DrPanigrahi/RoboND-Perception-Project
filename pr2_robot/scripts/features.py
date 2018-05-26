import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *


def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized


def compute_histogram(channel, nbins, bins_range):
    # TODO: Compute histograms of the colors and surface normals
    hist_0 = np.histogram(channel[0], bins=nbins, range=bins_range)
    hist_1 = np.histogram(channel[1], bins=nbins, range=bins_range)
    hist_2 = np.histogram(channel[2], bins=nbins, range=bins_range)
    # TODO: Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((hist_0[0], hist_1[0], hist_2[0])).astype(np.float64)
    # TODO: Normalize the concatenated histogram features
    normed_features = hist_features / np.sum(hist_features)

    return normed_features


def channels(feature_vectors): 
        # TODO: Create component arrays for each feature vector
    ch_0 = [vector[0] for vector in feature_vectors]
    ch_1 = [vector[1] for vector in feature_vectors]
    ch_2 = [vector[2] for vector in feature_vectors]

    return ch_0, ch_1, ch_2


def compute_color_histograms(cloud, using_hsv=False, nbins=32, bins_range=(0, 256)):
    
    point_colors_list = []

    # TODO: Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    return compute_histogram(channels(point_colors_list), nbins, bins_range)


def compute_normal_histograms(normal_cloud, nbins=360/18, bins_range=(-1.0,1.0)):            
    # TODO: Convert point cloud to array
    fields = ('normal_x', 'normal_y', 'normal_z')
    normals = pc2.read_points(normal_cloud, field_names=fields, skip_nans=True)

    return compute_histogram(channels(normals), nbins, bins_range)
