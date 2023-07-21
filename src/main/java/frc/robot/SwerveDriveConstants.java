// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SwerveDriveConstants {
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double STEERING_GEAR_RATIO = 150.0/7.0;
    public static final double wheelBase = Units.inchesToMeters(24);
    public static final double trackWidth = Units.inchesToMeters(24);

    public static final double maxSpeedMetersPerSecond = 10;

    // Contains the configuration info for each swerve module. Will be used to identify specific swerve modules.
    public enum SwerveModules {
        FRONT_LEFT(null),
        FRONT_RIGHT(null),
        BACK_LEFT(null),
        BACK_RIGHT(null);
        public final SwerveModuleConfiguration config;
        private SwerveModules(SwerveModuleConfiguration config) {
            this.config = config;
        }
    }

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2)
    );
    

    public class SwerveModuleConfiguration {
        public int driveMotorID;
        public int turnMotorID;
        public int CANCoderID;
        public SwerveModuleConfiguration(int driveMotorID, int turnMotorID, int CANCoderID) {
            this.driveMotorID = driveMotorID;
            this.turnMotorID = turnMotorID;
            this.CANCoderID = CANCoderID;
        }
    }
}
