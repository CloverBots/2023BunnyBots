// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;


public class DriveToDistanceCommand extends CommandBase {
  private SwerveSubsystem swerveSubsystem;
  private final float distanceMeters;

  private PIDController driveDistanceController = new PIDController(1, 0, 0);

  /** Creates a new DriveToDistanceCommand. */
  public DriveToDistanceCommand(SwerveSubsystem swerveSubsystem, float distanceMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.distanceMeters = distanceMeters;
    driveDistanceController.setSetpoint(distanceMeters);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.resetOdometry(new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = driveDistanceController.calculate(swerveSubsystem.getPose().getX());
    System.out.println("x spee"+ xSpeed);
    swerveSubsystem.setSpeed(xSpeed, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveDistanceController.atSetpoint();
  }
}
