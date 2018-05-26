import numpy as np
import matplotlib.pyplot as plt
import cv2

# Define a function to generate clusters
def cluster_gen(n_clusters, pts_minmax=(10, 100), x_mult=(1, 4), y_mult=(1, 3), 
                x_off=(0, 50), y_off=(0, 50)):
    
    # n_clusters = number of clusters to generate
    # pts_minmax = range of number of points per cluster 
    # x_mult = range of multiplier to modify the size of cluster in the x-direction
    # y_mult = range of multiplier to modify the size of cluster in the y-direction
    # x_off = range of cluster position offset in the x-direction
    # y_off = range of cluster position offset in the y-direction
    
    # Initialize some empty lists to receive cluster member positions
    clusters_x = []
    clusters_y = []
    labels = []
    
    # Genereate random values given parameter ranges
    n_points = np.random.randint(pts_minmax[0], pts_minmax[1], n_clusters)
    x_multipliers = np.random.randint(x_mult[0], x_mult[1], n_clusters)
    y_multipliers = np.random.randint(y_mult[0], y_mult[1], n_clusters)
    x_offsets = np.random.randint(x_off[0], x_off[1], n_clusters)
    y_offsets = np.random.randint(y_off[0], y_off[1], n_clusters)
     
    # Generate random clusters given parameter values
    for idx, npts in enumerate(n_points):       
        xpts = np.random.randn(npts) * x_multipliers[idx] + x_offsets[idx]
        ypts = np.random.randn(npts) * y_multipliers[idx] + y_offsets[idx]
        clusters_x.append(xpts)
        clusters_y.append(ypts)
        labels.append(np.zeros_like(xpts) + idx)
    
    # Return cluster positions
    return clusters_x, clusters_y, labels

def main():
    '''

    Parameters to Tweak:
    There are lots of parameters that you can tweak and they will affect the data you generate 
    and the result you get from running cv2.kmeans(). Keep the following in mind as you experiment:

    1) n_clusters and k_clusters need not be the same number!
    2) In your call to cluster_gen() you can tweak pts_minmax, x_mult, y_mult, x_off and y_off 
    to change the size and shape of the clusters you generate.
    3) The criteria you set for your k-means algorithm, max_iter and epsilon will determine 
    when the algorithm stops iterating.
    
    '''

    # Generate some clusters!
    n_clusters = 50
    clusters_x, clusters_y, labels = cluster_gen(n_clusters, pts_minmax=(50, 80), x_mult=(3, 9), y_mult=(1, 5), 
                x_off=(0, 150), y_off=(0, 150))
    # Convert to a single dataset in OpenCV format
    data = np.float32((np.concatenate(clusters_x), np.concatenate(clusters_y))).transpose()

    # Define k-means parameters
    # Number of clusters to define
    k_clusters = 7
    # Maximum number of iterations to perform
    max_iter = 20
    # Accuracy criterion for stopping iterations
    epsilon = 0.1
    # Number of times the algorithm is executed using different initial labellings
    attempts = 10
    # Define criteria in OpenCV format
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, max_iter, epsilon)
    # Call k-means algorithm on your dataset
    compactness, label, center = cv2.kmeans(data, k_clusters, None, criteria, attempts, cv2.KMEANS_RANDOM_CENTERS)

    # Define some empty lists to receive k-means cluster points
    kmeans_clusters_x = []
    kmeans_clusters_y = []

    # Extract k-means clusters from output
    for idx in range (k_clusters):
        kmeans_clusters_x.append(data[label.ravel()==idx][:,0])
        kmeans_clusters_y.append(data[label.ravel()==idx][:,1])
        
    # Plot up a comparison of original clusters vs. k-means clusters
    fig = plt.figure(figsize=(12,6))
    plt.subplot(121)
    min_x = np.min(data[:, 0])
    max_x = np.max(data[:, 0])
    min_y = np.min(data[:, 1])
    max_y = np.max(data[:, 1])
    for idx, xpts in enumerate(clusters_x): 
        plt.plot(xpts, clusters_y[idx], 'o')
        plt.xlim(min_x, max_x)
        plt.ylim(min_y, max_y)
        plt.title('Original Clusters', fontsize=20)
    plt.subplot(122)
    for idx, xpts in enumerate(kmeans_clusters_x):
        plt.plot(xpts, kmeans_clusters_y[idx], 'o')
        plt.scatter(center[:,0], center[:,1], s = 80, c = 'r', marker = 's')
        plt.xlim(min_x, max_x)
        plt.ylim(min_y, max_y)
        plt.title('k-means Clusters', fontsize=20)
    fig.tight_layout()
    plt.subplots_adjust(left=0.03, right=0.98, top=0.9, bottom=0.05)
    plt.show()
    #plt.show(block=True)

if __name__ == "__main__":
    main()