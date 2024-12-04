import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import csv
# Example data as a Pandas DataFrame
data = pd.read_csv('coordinates.csv')
data = data[['x','y','class']]

data = data.drop_duplicates()
data = data.dropna()
threshold = 0.3

def delete_distant_points(data, threshold):
    remaining_data = []

    # Group data by class
    for class_label, group in data.groupby('class'):
        points = group[['x', 'y']].values
        distances = np.linalg.norm(points[:, np.newaxis, :] - points[np.newaxis, :, :], axis=2)
        # Find points that have at least one other point within the threshold
        keep_indices = [i for i in range(len(points)) if np.any((distances[i] > 0) & (distances[i] <= threshold))]

        # If no points satisfy the condition, retain only one point
        if not keep_indices and len(points) > 0:
            keep_indices = [0]  # Keep at least one point

        # Append the remaining points
        remaining_data.append(group.iloc[keep_indices])

    # Concatenate remaining points
    return pd.concat(remaining_data, ignore_index=True)

# Filter out distant points
filtered_data = delete_distant_points(data, threshold)


    # Combine the data from both dataframes (coordinates and lidar)
x = list(filtered_data['x'])   # First column for x values
y = list(filtered_data['y'])  # Second column for y values

    # Assuming class_id is in the third column of the data dataframe
class_id = list(filtered_data['class']) # Third column for class_id, adding 100 for lidar data

    # Create a DataFrame combining the x, y, and class_id
df = pd.DataFrame({'x': x, 'y': y, 'class': class_id})
file_path = 'filtered.csv'
    
#     # Open file in write mode
with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)

        # Write header
        writer.writerow(['x', 'y', 'class'])

        # Write each row of filtered data
        for index, row in filtered_data.iterrows():
            writer.writerow([row['x'], row['y'], row['class']])
    # Plotting with Seaborn
plt.figure(figsize=(10, 8))
sns.scatterplot(x='x', y='y', hue='class', data=filtered_data, palette='Set1', s=50, alpha=0.7)
plt.xlim(-100, 100)
plt.ylim(-100, 100)

    # Adding title and labels
plt.title("filtered Scatter plot of points colored by class_id")
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.legend(title="Class ID")
plt.show()

# # Plot the data
# def plot_points(original_data, filtered_data):
#     plt.figure(figsize=(10, 6))

#     # Plot original data
#     for class_label, group in original_data.groupby('class'):
#         plt.scatter(group['x'], group['y'], label=f"Original {class_label}", alpha=0.5)

#     # Plot filtered data
#     for class_label, group in filtered_data.groupby('class'):
#         plt.scatter(group['x'], group['y'], label=f"Filtered {class_label}", edgecolor='black', s=100)

#     plt.title("Original vs Filtered Points")
#     plt.xlabel("X")
#     plt.ylabel("Y")
#     plt.legend()
#     plt.grid(True)
#     plt.show()

# # Plot original and filtered points
# plot_points(data, filtered_data)
