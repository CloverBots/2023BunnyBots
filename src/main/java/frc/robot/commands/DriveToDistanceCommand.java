// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;


public class DriveToDistanceCommand extends CommandBase {
  private SwerveSubsystem swerveSubsystem;

  private PIDController driveDistanceControllerX = new PIDController(7.0, 0.25, 0.1);
  private PIDController driveDistanceControllerY = new PIDController(7.0, 0.25, 0.1);
  private PIDController rotationController = new PIDController(Math.PI, 0, 0);

  private Timer timer;
  private double timeout;
  /** Creates a new DriveToDistanceCommand. */
  public DriveToDistanceCommand(SwerveSubsystem swerveSubsystem, double dx, double dy, double dTheta, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.timer = new Timer();
    this.timeout = timeout;
    this.swerveSubsystem = swerveSubsystem;
    driveDistanceControllerX.setSetpoint(dx);
    driveDistanceControllerX.setTolerance(0.005);
    driveDistanceControllerY.setSetpoint(dy);
    driveDistanceControllerY.setTolerance(0.005);
    rotationController.setSetpoint(dTheta);
    rotationController.setTolerance(0.005);
    rotationController.enableContinuousInput(0, 2*Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Status", true);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = driveDistanceControllerX.calculate(swerveSubsystem.getPose().getX());
    xSpeed = Math.copySign(Math.min(Math.abs(xSpeed),1), xSpeed);

    double ySpeed = driveDistanceControllerY.calculate(swerveSubsystem.getPose().getY());
    ySpeed = Math.copySign(Math.min(Math.abs(ySpeed),1), ySpeed);

    double dTheta = rotationController.calculate(Units.degreesToRadians(swerveSubsystem.getHeading()));
    dTheta = Math.copySign(Math.min(Math.abs(dTheta),2), dTheta);
    swerveSubsystem.setSpeed(xSpeed, ySpeed, -dTheta, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    SmartDashboard.putBoolean("Status", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveDistanceControllerX.atSetpoint() && driveDistanceControllerY.atSetpoint()) 
    || (timer.get() >= timeout);
  
  }
}
