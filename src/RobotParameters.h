#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

namespace RobotParameters {
	static constexpr unsigned k_updateRate = 50; // Hz

	static constexpr bool   k_leftDriveMotorInverted = false;
    static constexpr double k_drivetrainTrimKv = 0.01; // range of -0.5 to 0.5
    static constexpr double k_wheelRad = 3; // in
    static constexpr double k_wheelTrack = 19.5 * 1.2; // in
    static constexpr double k_maxSpeed = 110; // in/s
    static constexpr double k_maxAccel = 100; // in/s^2
    static constexpr double k_maxDeccel = -50; // in/s^2
    static constexpr double k_maxCentripAccel = 50; // in/s^2
    static constexpr double k_cornerStiffCoeff = 0;
    static constexpr double k_driveMotorToEncoderGearRatioLow = 11; // gear ratio from drive motor to encoder in low gear
    static constexpr double k_driveMotorToEncoderGearRatioHigh = 25; // gear ratio from drive motor to encoder in high gear
    static constexpr double k_driveEncoderToWheelGearRatio = 12.0 / 20.0 * 1.0 / 3.0; // gear ratio from drive encoder to wheel
    static constexpr double k_wheelSlipNoiseRatio = 1.2; // wheel encoder noise ratio used to detect wheel slip

    static constexpr double k_driveMotorControllerKp = 0.2;
    static constexpr double k_driveMotorControllerKi = 0;
    static constexpr double k_driveMotorControllerKd = 0;
    static constexpr double k_driveMotorControllerKsf = 0.09;
    static constexpr double k_driveMotorControllerKv = 0.00055;
    static constexpr double k_driveMotorControllerKap = 0.00015;
    static constexpr double k_driveMotorControllerKan = 0.00005;

    static constexpr double k_pathFollowerTimeoutAllowance = 0.5; // timeout path if takes longer than total path time plus this allowance (s)
    static constexpr double k_pathFollowerLatDistKp = 0;
    static constexpr double k_pathFollowerLatDistKi = 0;
    static constexpr double k_pathFollowerLatDistKd = 0;
    static constexpr double k_pathFollowerYawRateKp = 0;
    static constexpr double k_pathFollowerYawRateKi = 0;
    static constexpr double k_pathFollowerYawRateKd = 0;

    static constexpr double k_rotateToAngleTimeoutAllowance = 0.5; // timeout path if takes longer than total path time plus this allowance (s)
    static constexpr double k_rotateToAngleMaxYawRate = 0;
    static constexpr double k_rotateToAngleMaxAngAccel = 0;
    static constexpr double k_rotateToAngleMaxAngDeccel = 0;
    static constexpr double k_rotateToAngleTargetZone = 5;

    static constexpr unsigned k_ctreMagEncoderTicksPerRev = 4096;
    static constexpr unsigned k_grayhillEncoderTicksPerRev = 512;
};

#endif // ROBOT_PARAMETERS_H
