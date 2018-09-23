import csv
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.image as mpimg
import argparse

# read csv file name
parser = argparse.ArgumentParser()
parser.add_argument("csvFileName", help="path to csv file to plot")
args = parser.parse_args()

fieldFileName = '2018FieldTopDown.png' # field image file name
fieldWidth = 12.0 * 54.0 / 2.0  # field length (in)
fieldLength = 12.0 * 54.0 # field length (in)

class waypoint:
    def __init__(self):
        self.xPos = 0
        self.yPos = 0
        self.speed = 0
        self.maxDistThresh = 0

# read csv
with open(args.csvFileName) as csvFile:
    csvReader = csv.reader(csvFile, delimiter=',')

    # skip headers
    for i in range(9):
        next(csvReader)

    # read waypoints
    waypoints = []
    for row in csvReader:
        tempWaypoint = waypoint()
        tempWaypoint.xPos = float(row[0])
        tempWaypoint.yPos = float(row[1])
        tempWaypoint.speed = float(row[2])
        tempWaypoint.maxDistThresh = float(row[3])
        waypoints.append(tempWaypoint)

# plot waypoints
fig, ax = plt.subplots()
fig.canvas.set_window_title('Waypoints')

waypointsXPos = []
waypointsYPos = []
for i in range(len(waypoints)):
    waypointsXPos.append(waypoints[i].xPos)
    waypointsYPos.append(waypoints[i].yPos)
plt.plot(waypointsXPos, waypointsYPos, 'bx-')

# plot waypoints max dist threshold
for i in range(len(waypoints)):
    patch = patches.Circle((waypoints[i].xPos, waypoints[i].yPos), waypoints[i].maxDistThresh, fill=False,
                            edgecolor='r', linestyle='--')
    ax.add_patch(patch)

# plot field
img = mpimg.imread(fieldFileName)
plt.imshow(img, aspect='equal', extent=(0, fieldWidth, 0, fieldLength))

ax.set_xlim(0, fieldWidth)
ax.set_ylim(0, fieldLength)
ax.set_aspect('equal')
ax.set_xlabel('x (in)')
ax.set_ylabel('y (in)')

plt.show()
