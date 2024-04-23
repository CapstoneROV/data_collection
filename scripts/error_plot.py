# import numpy as np

# # Sample data: 10 rows, 6 columns (3 velocities, 3 positions)
# data = np.random.rand(10, 6) * 100  # Random data scaled by 100 for variability

# # Save to a text file
# np.savetxt('example_array.txt', data, fmt='%0.2f')

import numpy as np
import sys
import sys
import matplotlib.pyplot as plt

def read_data(filepath):
    """Read the N by 6 array from the text file."""
    return np.loadtxt(filepath, delimiter=',')

def calculate_error(data, desired_point):
    """Calculate the Euclidean distance from each point in the array to the desired point."""
    positions = data[:, 0:3] # Get the xpos, ypos, zpos columns (first 3 points)
    time = data[:, 3] # Get the time_boot_ms column
    errors = np.sqrt(((positions - desired_point) ** 2).sum(axis=1))
    print("Reached Point: ", positions[-1]);
    print("Desired Point: ", desired_point);
    print("Time Taken: ", time[-1]-time[0]);
    return errors, time

def plot_errors(errors, time):
    """Plot the errors using matplotlib and annotate the minimum error."""
    plt.figure(figsize=(10, 5))
    plt.plot(time, errors, marker='o', markersize=1)
    plt.title('Euclidean Distance Between Each Point and Desired Point')
    plt.xlabel('Time(ms)')
    plt.ylabel('Distance')
    plt.grid(True)

    # Find and annotate the minimum error
    min_error_index = np.argmin(errors)
    min_error = errors[min_error_index]
    min_error_time = time[min_error_index]
    plt.annotate(f'Min error: {min_error:.2f}', xy=(min_error_time, min_error), xytext=(min_error_time, min_error+5),
                 arrowprops=dict(facecolor='red', shrink=0.05), fontsize=12, color='red')

    # plt.show()
    plt.savefig('errors_plot.png')

def print_minimum_error(errors):
    """Print the minimum error and its index."""
    min_error_index = np.argmin(errors)
    min_error = errors[min_error_index]
    print(f"The minimum error is {min_error:.2f} at index {min_error_index}")
# Main execution
if __name__ == "__main__":

    # Accept file path and desired point as input parameters
    if len(sys.argv) < 5:
        print("Please provide the data file path and desired point (x, y, z) as command line arguments.")
        sys.exit(1)
    file_path = sys.argv[1]
    desired_point = np.array([float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])])

    # Read the data from file
    data = read_data(file_path)

    # Calculate errors
    errors, time = calculate_error(data, desired_point)

    # Plot errors with annotation
    plot_errors(errors, time)

    # Print minimum error
    print_minimum_error(errors)
