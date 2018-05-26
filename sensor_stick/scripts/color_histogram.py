import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


 

# Define a function to compute color histogram features  
def rgb_color_hist(rgb_img, nbins=32, bins_range=(0, 256)):
    # Compute the histogram of the HSV channels separately
    r_hist = np.histogram(rgb_img[:,:,0], bins=nbins, range=bins_range)
    g_hist = np.histogram(rgb_img[:,:,1], bins=nbins, range=bins_range)
    b_hist = np.histogram(rgb_img[:,:,2], bins=nbins, range=bins_range)

    # Generating bin centers
    # r_hist[0] contains the counts in each of the bins
    # r_hist[1] contains the bin edges (so it is one element longer than r_hist[0])
    bin_edges = r_hist[1]
    bin_centers = (bin_edges[1:]  + bin_edges[0:len(bin_edges)-1])/2

    # Plot a figure with all three bar charts
    fig = plt.figure(figsize=(12,3))
    plt.subplot(131)
    plt.bar(bin_centers, r_hist[0])
    plt.xlim(0, 256)
    plt.title('R Histogram')
    plt.subplot(132)
    plt.bar(bin_centers, g_hist[0])
    plt.xlim(0, 256)
    plt.title('G Histogram')
    plt.subplot(133)
    plt.bar(bin_centers, b_hist[0])
    plt.xlim(0, 256)
    plt.title('B Histogram')
    plt.show()

    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((r_hist[0], g_hist[0], b_hist[0])).astype(np.float64)
    # Normalize the result
    norm_features = hist_features / np.sum(hist_features)

    # Plot a figure with all three bar charts
    if norm_features is not None:
        fig = plt.figure(figsize=(8,4))
        plt.plot(norm_features)
        plt.title('RGB Feature Vector', fontsize=15)
        plt.tick_params(axis='both', which='major', labelsize=15)
        fig.tight_layout()
        plt.show()
    else:
        print('Your function is returning None...')

    # Return the feature vector
    return norm_features

# Define a function to compute color histogram features  
def hsv_color_hist(img, nbins=32, bins_range=(0, 256)):
    '''
    HSV
     - Hue (angular position around the cylinder)           -> Pixel Color
     - Saturation (radial distance from the center axis)    -> Intensity of the Color
     - Value (along the vertical axis)                      -> Overall Brightness
    '''
    # Convert from RGB to HSV using cv2.cvtColor()
    hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # Compute the histogram of the HSV channels separately
    h_hist = np.histogram(hsv_img[:,:,0], bins=nbins, range=bins_range)
    s_hist = np.histogram(hsv_img[:,:,1], bins=nbins, range=bins_range)
    v_hist = np.histogram(hsv_img[:,:,2], bins=nbins, range=bins_range)

    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((h_hist[0], s_hist[0], v_hist[0])).astype(np.float64)
    # Normalize the result
    norm_features = hist_features / np.sum(hist_features)

    # Return the feature vector
    return norm_features


def main():
    # Read in an image
    image = mpimg.imread('udacican.png')
    # Your other options for input images are:
    # hammer.jpeg
    # beer.jpeg
    # bowl.jpeg
    # create.jpeg
    # disk_part.jpeg

    rgb_feature_vec = rgb_color_hist(image, nbins=32, bins_range=(0, 256))
    hsv_feature_vec = hsv_color_hist(image, nbins=32, bins_range=(0, 256))

    # Plot a figure with all three bar charts
    fig = plt.figure(figsize=(12,6))
    plt.subplot(211)
    plt.plot(rgb_feature_vec)
    plt.title('RGB Feature Vector Histogram', fontsize=18)
    plt.tick_params(axis='both', which='major', labelsize=10)
    plt.subplot(212)
    plt.plot(hsv_feature_vec)
    plt.title('HSV Feature Vector Histogram', fontsize=18)
    plt.tick_params(axis='both', which='major', labelsize=10)
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()