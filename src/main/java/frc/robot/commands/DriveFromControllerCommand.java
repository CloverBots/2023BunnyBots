package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveFromControllerCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private final double CONTROLLER_DEADZONE = 0.05;

    public DriveFromControllerCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(SwerveDriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(SwerveDriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(SwerveDriveConstants.teleOpMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Get real-time controller inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // Apply the deadzone. This will prevent the robot from moving at very small values
        xSpeed = Math.abs(xSpeed) > CONTROLLER_DEADZONE ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > CONTROLLER_DEADZONE ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > CONTROLLER_DEADZONE ? turningSpeed : 0.0;


        xSpeed = Math.pow(xSpeed, 3);
        ySpeed = Math.pow(ySpeed, 3);
        turningSpeed = Math.pow(turningSpeed, 5);

        // Limit the acceleration for moving and rotation using the rate limiters
        // Using the rate limited value from 0 to 1, this will make a value of 1 move the robot at the configured maximum speed
        xSpeed = xLimiter.calculate(xSpeed) * SwerveDriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;
        ySpeed = yLimiter.calculate(ySpeed) * SwerveDriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * SwerveDriveConstants.teleOpMaxAngularSpeed;

        // Construct a ChassisSpeeds object, which will contain the movement and rotation speeds that we want our robot to do.
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Driving will be relative to field.
            // If this is enabled, then pressing forward will always move the robot forward, no matter its rotation.
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    ySpeed, xSpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Driving will be relative to the robot.
            // If this is enabled, then pressing forward will move the robot in the direction that it is currently facing.
            chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
        }

        // This will take the speeds that we want our robot to move and turn at, and calculate the required direction and speed for each swerve module on the robot.
        SwerveModuleState[] moduleStates = SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Set the swerve modules to their required states.
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}