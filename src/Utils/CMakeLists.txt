cmake_minimum_required(VERSION 3.12.1)

project(utilsTest)

set(SOURCES main.cpp
            Rotation2D.cpp
            Translation2D.cpp
            Pose2D.cpp
            Interpolate.cpp
            TankDrivePathGenerator.cpp
            PIDVAController.cpp
            TrajectoryGenerator1D.cpp
            TankDriveKinematics.cpp
            NormalizeToRange.cpp
)

include_directories(include)

add_executable(utilsTest ${SOURCES})
