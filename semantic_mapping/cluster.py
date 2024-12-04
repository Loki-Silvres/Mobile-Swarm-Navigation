import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import pandas as pd
from matplotlib.colors import ListedColormap
import csv
# Read the data from CSV
data = pd.read_csv('filtered.csv')

# Remove duplicates
data = data.drop_duplicates()

# Define the classes you want to cluster
classes = data['class'].unique()

# Create a plot with a white background
plt.figure(figsize=(12, 8), facecolor='white')

# Create a color map with a vibrant color palette
color_map = ListedColormap(plt.cm.Set2.colors[:len(classes)])

# Loop through each class to process and cluster its coordinates
for i, class_id in enumerate(classes):
    # Filter data for the current class
    
    coordinates = data[data['class'] == class_id]
    width = data['z'].max()
    coordinates = coordinates.dropna()

    # Select only the 'x' and 'y' columns for clustering
    coordinates = coordinates[['x', 'y']]
    # Convert the DataFrame to a NumPy array
    coordinates = coordinates.to_numpy()

    # Apply DBSCAN
    dbscan = DBSCAN(eps=0.4, min_samples=2)  # Adjust eps as needed
    labels = dbscan.fit_predict(coordinates)

    # Filter out noise points (where label is -1)
    filtered_coordinates = coordinates[labels != -1]
    filtered_labels = labels[labels != -1]

    # Visualize clusters for the current class
    unique_labels = np.unique(filtered_labels)

    # Loop over each cluster label in the filtered data
    for label in unique_labels:
        # Get coordinates for the current cluster
        cluster_points = filtered_coordinates[filtered_labels == label]

        # Calculate the bounding box dimensions
        min_x, min_y = np.min(cluster_points, axis=0)
        max_x, max_y = np.max(cluster_points, axis=0)

        # Ensure the box is square by taking the larger side and centering
        side_length_x = max_x - min_x
        side_length_y = max_y - min_y
        
        if(side_length_x==0):
            side_length_x = 1
        elif(side_length_y==0):
            side_length_y = 1    
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        min_x = center_x - side_length_x / 2
        max_x = center_x + side_length_x / 2
        min_y = center_y - side_length_y / 2
        max_y = center_y + side_length_y / 2
        with open('semanticmap.csv', mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([min_x,min_y,side_length_x,side_length_y])
        # Draw a square around the cluster
        param = 0.8
        if(side_length_x == max(side_length_x,side_length_y) and max(side_length_x,side_length_y)<0.8*width):
            side_length_x=width
        elif(side_length_y==max(side_length_x,side_length_y) and max(side_length_x,side_length_y)<0.8*width):
            side_length_y = width
        plt.gca().add_patch(
            plt.Rectangle((min_x, min_y), side_length_x, side_length_y, 
                          color=color_map(i), alpha=1, edgecolor='black', linewidth=1.5)
        )

        # Label the square with the class ID and cluster number
        plt.text(center_x, center_y, f"{class_id}-{label}", fontsize=9, ha='center', va='center', color='black')

# Title and labels with a larger font
plt.title("Clusters of Detected Objects (Square Boxes)", fontsize=16, fontweight='bold', color='darkblue')
plt.xlabel("X Coordinate", fontsize=12, fontweight='bold')
plt.ylabel("Y Coordinate", fontsize=12, fontweight='bold')

# Adjust axis limits to provide enough space around the points
plt.xlim(-30, 30)
plt.ylim(-30, 30)

# Make grid lines lighter and add background color
plt.grid(True, which='both', linestyle='--', linewidth=0.5, color='gray')
plt.gca().set_facecolor('lightgray')

# Add a legend for classes with a clear style
handles, labels = plt.gca().get_legend_handles_labels()
by_label = dict(zip(labels, handles))  # Avoid duplicate labels
plt.legend(by_label.values(), by_label.keys(), title="Classes", loc='upper right', fontsize=10, title_fontsize=12, frameon=True)

# Display the plot with an attractive layout
plt.tight_layout()
plt.show()
