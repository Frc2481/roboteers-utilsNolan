#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

class RobotParameters {
public:
	static constexpr unsigned k_updateRate = 20; // Hz

    static constexpr double k_wheelRad = 3; // in
    static constexpr double k_wheelTrack = 20; // in
    static constexpr double k_maxSpeed = 100; // in/s
    static constexpr double k_maxAccel = 100; // in/s^2
    static constexpr double k_maxDeccel = -100; // in/s^2
    static constexpr double k_maxCentripAccel = 20; // in/s^2
    static constexpr double k_cornerStiffCoeff = 0;
    static constexpr double k_driveGearRatio = 1; // gear ratio from encoder to wheel
    static constexpr double k_wheelSlipNoiseRatio = 1.2 // wheel encoder noise ratio used to detect wheel slip

    static constexpr double k_driveMotorControllerKp = 1;
    static constexpr double k_driveMotorControllerKi = 0;
    static constexpr double k_driveMotorControllerKd = 0;
    static constexpr double k_driveMotorControllerKv = 0;
    static constexpr double k_driveMotorControllerKa = 0;

    static constexpr double k_pathFollowerTimeoutAllowance = 0.5; // timeout path if takes longer than total path time plus this allowance (s)
    static constexpr double k_pathFollowerKpTurn = 1;

    static constexpr double k_rotateToAngleControllerKp = 1;
    static constexpr double k_rotateToAngleControllerKi = 0;
    static constexpr double k_rotateToAngleControllerKd = 0;
    static constexpr double k_rotateToAngleControllerKv = 0;
    static constexpr double k_rotateToAngleControllerKa = 0;
    static constexpr double k_rotateToAngleControllerTargetZone = 1; // deg
    static constexpr double k_rotateToAngleControllerTargetZoneDebounce = 1; // s

    static constexpr unsigned k_ctreMagEncoderTicksPerRev = 4096;
    static constexpr unsigned k_grayhillEncoderTicksPerRev = 512;
};

#endif // ROBOT_PARAMETERS_H