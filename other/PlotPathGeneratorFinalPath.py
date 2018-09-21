import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

csvFileName = 'PathGeneratorFinalPath.csv' # csv file name
fieldFileName = '2018FieldTopDown.png' # field image file name
fieldWidth = 12.0 * 54.0 / 2.0  # field length (in)
fieldLength = 12.0 * 54.0 # field length (in)
robotWidth = 24 # robot width (in)
robotLength = 36 # robot length (in)

def robotPerimeter(width, length):
    frCorner = np.matrix([[width / 2.0], [length / 2.0]])
    brCorner = np.matrix([[width / 2.0], [-length / 2.0]])
    blCorner = np.matrix([[-width / 2.0], [-length / 2.0]])
    flCorner = np.matrix([[-width / 2.0], [length / 2.0]])

    return np.block([frCorner, brCorner, blCorner, flCorner, frCorner])

class pathPoint:
    def __init__(self):
        self.time = 0
        self.xPos = 0
        self.yPos = 0
        self.yaw = 0
        self.dist = 0
        self.vel = 0
        self.accel = 0
        self.yawRate = 0

def rotMat2D(angDeg):
    angRad = np.deg2rad(angDeg)
    return np.matrix([[np.cos(angRad), -np.sin(angRad)],
                      [np.sin(angRad), np.cos(angRad)]])

# read csv
with open(csvFileName) as csvFile:
    csvReader = csv.reader(csvFile, delimiter=',')

    # skip headers
    for i in range(1):
        next(csvReader)

    # read path points
    pathPoints = []
    for row in csvReader:
        tempPathPoint = pathPoint()
        tempPathPoint.time = float(row[0])
        tempPathPoint.xPos = float(row[1])
        tempPathPoint.yPos = float(row[2])
        tempPathPoint.yaw = float(row[3])
        tempPathPoint.dist = float(row[4])
        tempPathPoint.vel = float(row[5])
        tempPathPoint.accel = float(row[6])
        tempPathPoint.yawRate = float(row[7])
        pathPoints.append(tempPathPoint)

# plot final path
fig, ax = plt.subplots()
fig.canvas.set_window_title('Final Path')

for i in range(len(pathPoints)):
    robotCenter = np.matrix([[pathPoints[i].xPos], [pathPoints[i].yPos]])
    robotPerimeterTrans = robotCenter + rotMat2D(pathPoints[i].yaw) * robotPerimeter(robotWidth, robotLength)
    plt.plot(robotPerimeterTrans[0, :].transpose(), robotPerimeterTrans[1, :].transpose(), 'b-')

# plot field
img = mpimg.imread(fieldFileName)
plt.imshow(img, aspect='equal', extent=(0, fieldWidth, 0, fieldLength))

ax.set_xlim(0, fieldWidth)
ax.set_ylim(0, fieldLength)
ax.set_aspect('equal')
ax.set_xlabel('x (in)')
ax.set_ylabel('y (in)')

plt.show()

# plot final path dynamics
fig, ax = plt.subplots(3, 1, sharex='col')
fig.canvas.set_window_title('Final Path Dynamics')

pathPointsTime = []
pathPointsDist = []
pathPointsVel = []
pathPointsAccel = []
for i in range(len(pathPoints)):
    pathPointsTime.append(pathPoints[i].time)
    pathPointsDist.append(pathPoints[i].dist)
    pathPointsVel.append(pathPoints[i].vel)
    pathPointsAccel.append(pathPoints[i].accel)

# plot pos
plt.subplot(3, 1, 1)
plt.plot(pathPointsTime, pathPointsDist)
plt.xlabel('time (s)')
plt.ylabel('dist (in)')

# plot vel
plt.subplot(3, 1, 2)
plt.plot(pathPointsTime, pathPointsVel)
plt.xlabel('time (s)')
plt.ylabel('vel (in/s)')

# plot accel
plt.subplot(3, 1, 3)
plt.plot(pathPointsTime, pathPointsAccel)
plt.xlabel('time (s)')
plt.ylabel('accel (in/s^2)')

plt.show()

