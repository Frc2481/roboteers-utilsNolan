cmake_minimum_required(VERSION 3.12.1)

project(frcMain)

set(SOURCES src/main.cpp
            src/Rotation2D.cpp
            src/Translation2D.cpp
            src/RigidTransform2D.cpp
            src/Interpolate.cpp
            src/PathGenerator.cpp
    )

include_directories(include
                    )

add_executable(frcMain ${SOURCES})
