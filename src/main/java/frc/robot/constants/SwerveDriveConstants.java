// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Constant values relating to the Swerve Drive */
public class SwerveDriveConstants {
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;

    /** The driving gear ratio for the swerve module (MK4i L3). This is how many times the drive motor has to turn in order for the wheel to make 1 rotation. */
    public static final double DRIVE_GEAR_RATIO = 6.12;

    /** The driving gear ratio for the Swerve Module (MK4i). This is how many times the turning motor has to turn in order for the module to make 1 full rotation. */
    public static final double TURNING_GEAR_RATIO = 150.0/7.0;

    // Length of the robot chassis, front to back
    public static final double wheelBase = Units.inchesToMeters(24);
    // Width of the robot chassis, left to right
    public static final double trackWidth = Units.inchesToMeters(24);

    /** The maximum speed of the robot, in meters per second during TeleOp. Use this to limit the speed when using a controller.*/
    public static final double TELEOP_MAX_SPEED_METERS_PER_SECOND = 1;

    /** The PHYSICAL maximum speed of the robot, if all motors were running at max power. */
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 10;

    /** Maximum speed for the robot's turning. */
    public static final double teleOpMaxAngularSpeed = 1 * (2 * Math.PI);
    /** The maximum angular acceleration for the robot's turning. */
    public static final double teleOpMaxAngularAccelerationUnitsPerSecond = 3;

    /** Multiply the output of {@code getSelectedSensorPosition()} by this to get the total distance travelled, in meters, on a swerve module. */
    public static final double DRIVE_ENCODER_TO_METERS = (WHEEL_CIRCUMFERENCE / (DRIVE_GEAR_RATIO * 2048.0));

    /** Multiply the output of {@code getSelectedSensorVelocity()} by this to get the current velocity, in meters per second, on a swerve module. */
    public static final double DRIVE_ENCODER_VELOCITY_TO_METERS_PER_SECOND = (600.0 * WHEEL_CIRCUMFERENCE) / (2048.0 * 60 * DRIVE_GEAR_RATIO);

    // Used for position and velocity conversions on the NEO turning motor on the swerve modules.
    public static final double TURNING_ENCODER_TO_RAD = (2 * Math.PI) / TURNING_GEAR_RATIO;
    public static final double TURNING_ENCODER_TO_DEG = (360) / TURNING_GEAR_RATIO;
    public static final double TURNING_ENCODER_TO_RADS_PER_SECOND = TURNING_ENCODER_TO_RAD / 60;

    public static final double kPTurning = 0.2;
    public static final double kITurning = 0;
    public static final double kDTurning = 0;

    /**
     * Contains the configuration info for each swerve module.
     */
    public static enum SwerveModuleConfigurations {
        FRONT_LEFT(10, 14, 18, 0),
        FRONT_RIGHT(11, 15, 19, 0),
        BACK_LEFT(12, 16, 20, 0),
        BACK_RIGHT(13, 17, 21, 0);

        public int driveMotorID;
        public int turnMotorID;
        public int CANCoderID;
        public double encoderOffset;

        private SwerveModuleConfigurations(int driveMotorID, int turnMotorID, int CANCoderID, double encoderOffset) {
            this.driveMotorID = driveMotorID;
            this.turnMotorID = turnMotorID;
            this.CANCoderID = CANCoderID;
            this.encoderOffset = encoderOffset;
        }
    }

    /**
     * This calculates the exact speed and rotation of every swerve module needed to make the robot go in a specific direction and rotation.
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2)
    );
}
