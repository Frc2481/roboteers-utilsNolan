cmake_minimum_required(VERSION 3.12.1)

project(frcMain)

set(SOURCES src/main.cpp
            src/Rotation2D.cpp
            src/Translation2D.cpp
            src/Pose2D.cpp
            src/Interpolate.cpp
            src/TankDrivePathGenerator.cpp
            src/PIDVAController.cpp
            src/TrajectoryGenerator1D.cpp
            src/TankDriveKinematics.cpp
            src/TankDrivePose.cpp
)

include_directories(include)

add_executable(frcMain ${SOURCES})
