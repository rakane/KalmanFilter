#!/usr/bin/env python3

import matplotlib.pyplot as plt
import sys

# Check input arguments
numArgs = len(sys.argv)
if numArgs != 2:
    print("Failed to execute! usage: python3 plotResiduals.py <residuals sum>")
    print(numArgs)
    exit(1)

# Get input arguments, points passed as comma separated list
residualPoints = sys.argv[1].split(",")

# Convert to floats
for i in range(len(residualPoints)):
    residualPoints[i] = float(residualPoints[i])

# Create time points
timePoints = []
for i in range(len(residualPoints)):
    timePoints.append(i)

# print("Residual points: ", residualPoints)

# Plot
plt.plot(timePoints, residualPoints)
plt.xscale("linear")
plt.yscale("linear")

plt.xlabel("Time (s)")
plt.ylabel("Position Residuals Sum")
plt.title("Position Residuals Sum vs. Time")
plt.legend(["Truth", "Filter"])

# Show plot, non-blocking
plt.show()
