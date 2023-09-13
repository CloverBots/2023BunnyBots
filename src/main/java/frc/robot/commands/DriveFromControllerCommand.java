package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveFromControllerCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, crawlTrigger;
    private final Supplier<Boolean> yButtSupplier, bButtSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented = true;
    private final double CONTROLLER_DEADZONE = 0.05;
    private boolean lastPressedBoth = false;
    private double maxSpeed = SwerveDriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;

    public DriveFromControllerCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> yButtSupplier,
            Supplier<Boolean> bButtSupplier,
            Supplier<Double> crawlTrigger,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.yButtSupplier = yButtSupplier;
        this.bButtSupplier = bButtSupplier;
        this.crawlTrigger = crawlTrigger;
        this.xLimiter = new SlewRateLimiter(SwerveDriveConstants.teleOpMaxAccelerationMetersPerSecond);
        this.yLimiter = new SlewRateLimiter(SwerveDriveConstants.teleOpMaxAccelerationMetersPerSecond);
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

        double speedMultiplier = crawlTrigger.get() >= 0.5 ? 0.05 : 1;
        
        double magnitude = Math.hypot(ySpeed, xSpeed);

        magnitude = Math.pow(magnitude, 3);

        turningSpeed = Math.pow(turningSpeed, 5);

        magnitude = Math.min(magnitude, 1);
        // Limit the acceleration for moving and rotation using the rate limiters
        // Using the rate limited value from 0 to 1, this will make a value of 1 move the robot at the configured maximum speed
        magnitude = xLimiter.calculate(magnitude) * SwerveDriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;
        
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * SwerveDriveConstants.teleOpMaxAngularSpeed;
        
        magnitude *= speedMultiplier;
        xSpeed *= magnitude;
        ySpeed *= magnitude;

        turningSpeed *= speedMultiplier;
        if (yButtSupplier.get() && bButtSupplier.get() && lastPressedBoth == false) {
            fieldOriented = !fieldOriented;
        }
        lastPressedBoth = yButtSupplier.get() && bButtSupplier.get();
        SmartDashboard.putBoolean("Field Oriented", fieldOriented);
        swerveSubsystem.setSpeed(ySpeed, xSpeed, turningSpeed, fieldOriented);
        
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