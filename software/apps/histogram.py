import matplotlib.pyplot as plt
import numpy as np

# Read the file, skipping the header line
with open('output.txt', 'r') as file:
    # Skip the first line (header)
    next(file)
    
    # Read the remaining lines and convert to numbers
    numbers = []
    for line in file:
        try:
            numbers.append(float(line.strip()))
        except ValueError:
            # Skip lines that don't contain valid numbers
            continue

# Create the histogram
plt.figure(figsize=(10, 6))
plt.hist(numbers, bins=16, edgecolor='black', alpha=0.7)
plt.title('Histogram of Numbers from output.txt')
plt.xlabel('Value')
plt.ylabel('Frequency')
plt.grid(axis='y', alpha=0.75)

# Add a bit more style
#plt.axvline(np.mean(numbers), color='red', linestyle='dashed', linewidth=1, label=f'Mean: {np.mean(numbers):.2f}')
#plt.legend()

# Show the plot
plt.tight_layout()
plt.show()