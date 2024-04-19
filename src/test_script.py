# import numpy as np

# # Sample data: 10 rows, 6 columns (3 velocities, 3 positions)
# data = np.random.rand(10, 6) * 100  # Random data scaled by 100 for variability

# # Save to a text file
# np.savetxt('example_array.txt', data, fmt='%0.2f')

import numpy as np
import matplotlib.pyplot as plt

def read_data(filepath):
    """Read the N by 6 array from the text file."""
    return np.loadtxt(filepath)

def calculate_error(data, desired_point):
    """Calculate the Euclidean distance from each point in the array to the desired point."""
    positions = data[:, 3:]  # Get the xpos, ypos, zpos columns
    errors = np.sqrt(((positions - desired_point) ** 2).sum(axis=1))
    return errors

def plot_errors(errors):
    """Plot the errors using matplotlib and annotate the minimum error."""
    plt.figure(figsize=(10, 5))
    plt.plot(errors, marker='o')
    plt.title('Error Between Each Point and Desired Point')
    plt.xlabel('Index of Point')
    plt.ylabel('Error')
    plt.grid(True)

    # Find and annotate the minimum error
    min_error_index = np.argmin(errors)
    min_error = errors[min_error_index]
    plt.annotate(f'Min error: {min_error:.2f}', xy=(min_error_index, min_error), xytext=(min_error_index, min_error+5),
                 arrowprops=dict(facecolor='red', shrink=0.05), fontsize=12, color='red')

    plt.show()

def print_minimum_error(errors):
    """Print the minimum error and its index."""
    min_error_index = np.argmin(errors)
    min_error = errors[min_error_index]
    print(f"The minimum error is {min_error:.2f} at index {min_error_index}")

# Main execution
if __name__ == "__main__":
    # Read the data from file
    data = read_data('example_array.txt')

    # Desired point (x, y, z)
    desired_point = np.array([50, 50, 50])  # Example point

    # Calculate errors
    errors = calculate_error(data, desired_point)

    # Plot errors with annotation
    plot_errors(errors)

    # Print minimum error
    print_minimum_error(errors)
