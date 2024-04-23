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
    """Read the N by 4 array from the text file."""
    return np.loadtxt(filepath, delimiter=',')

# Main execution
if __name__ == "__main__":

    # Accept file path and desired point as input parameters
    if len(sys.argv) < 2:
        print("Please provide the data file path as command line arguments.")
        sys.exit(1)
    file_path = sys.argv[1]
    print(file_path)
    # Read the data from file
    data = read_data(file_path);
    positions = data[:, 0:3]; # (N, 3) array of positions
    x_pos = positions[:, 0];
    time = data[:, 3];
    # Plot X axis
    plt.figure(figsize=(10, 5))
    plt.plot(time, x_pos, marker='o', markersize=1)
    plt.title('X Position vs Time')
    plt.xlabel('Time(ms)')
    plt.ylabel('X Position')
    plt.grid(True)
    plt.savefig('x_position_plot.png')
