#!/usr/bin/env python3

import matplotlib.pyplot as plt
import sys

# Check input arguments
numArgs = len(sys.argv)

# Get input arguments, points passed as comma separated list
numDimensions = int(sys.argv[1])

if numArgs != 2 + numDimensions * 3:
    print(
        "Failed to execute! usage: python3 plot.py <num dimensions> <truth DIM_1 points> <filter DIM_1 points> <measurement DIM_1 points> ... <truth DIM_N points> <filter DIM_N points> <measurement DIM_N points>"
    )
    print(numArgs)
    exit(1)


truthPoints = []
kalmanFilterPoints = []
measurementPoints = []

for i in range(numDimensions):
    truthPointsDim = sys.argv[2 + i * 3].split(",")
    kalmanFilterPointsDim = sys.argv[3 + i * 3].split(",")
    measurementPointsDim = sys.argv[4 + i * 3].split(",")

    # Ensure lengths are equal
    if len(truthPointsDim) != len(kalmanFilterPointsDim) or len(
        measurementPointsDim
    ) != len(kalmanFilterPointsDim):
        print("Error: truth and filter points must be the same length")
        exit(1)

    # Convert to floats
    for i in range(len(truthPointsDim)):
        truthPointsDim[i] = float(truthPointsDim[i])
        kalmanFilterPointsDim[i] = float(kalmanFilterPointsDim[i])
        measurementPointsDim[i] = float(measurementPointsDim[i])

    truthPoints.append(truthPointsDim)
    kalmanFilterPoints.append(kalmanFilterPointsDim)
    measurementPoints.append(measurementPointsDim)

# Create time points
timePoints = []
for i in range(len(truthPoints[0])):
    timePoints.append(i)

# Debug values
# print("Truth points: ", truthPoints)
# print("Filter points: ", kalmanFilterPoints)
# print("Measurement points: ", measurementPoints)

# Plot subplots
fig = plt.figure()
fig.suptitle("Constant acceleration filter vs. truth")

axs = []

for i in range(numDimensions):
    axs.append(fig.add_subplot(numDimensions, 1, i + 1))

    labelStr = "Dimension " + str(i + 1)

    axs[i].plot(timePoints, truthPoints[i])
    axs[i].plot(timePoints, kalmanFilterPoints[i])
    axs[i].plot(timePoints, measurementPoints[i], "o")
    axs[i].set_xscale("linear")
    axs[i].set_yscale("linear")
    axs[i].set_xlabel("Time (s)")
    axs[i].set_ylabel(labelStr)
    axs[i].legend(["Truth", "Filter", "Measurement"])

fig.show()
plt.show()
