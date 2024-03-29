// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C.Port;

/**
 * Contains IDs for the devices on the robot.</p>
 * <strong>To use the IDs for swerve drive components, use {@link SwerveDriveConstants}.</strong>
 */
public final class IDs {
  public static final int driverControllerPort = 0;
  public static final Port AHRS_PORT_ID = Port.kMXP;

  // Port IDs for the controllers
  public static final int CONTROLLER_DRIVE_PORT = 0;
  public static final int CONTROLLER_OPERATOR_PORT = 1;
  public static final int TALONSRX_MOTOR = 05; // Update when assigned
  public static final int RABBIT_DEPLOY_DEVICE = 20; //Update when assigned
  public static final int INTAKE_DEVICE = 22; // same
  public static final int BALL_DEPLOY = 21; // same
  public static final int SHOOTER_DEVICE = 20; // same
}