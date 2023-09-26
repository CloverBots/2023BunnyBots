package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveFromControllerCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, rightStickX, rightStickY, crawlTrigger, slowRotate;
    private final Supplier<Boolean> yButton, bButton, aButton, xButton;
    private final Supplier<Integer> dPad;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented = true;
    private boolean pointedTurning = false;
    private final double CONTROLLER_DEADZONE = 0.05;
    private boolean fieldOrientedCache, pointedModeCache = false;
    private PIDController rotationController = new PIDController(Math.PI, 0, 0);

    public DriveFromControllerCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> rightStickX,
            Supplier<Double> rightStickY,
            Supplier<Boolean> yButton,
            Supplier<Boolean> bButton,
            Supplier<Boolean> aButton,
            Supplier<Boolean> xButton,
            Supplier<Double> crawlTrigger,
            Supplier<Double> slowRotate,
            Supplier<Integer> dPad) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.rightStickX = rightStickX;
        this.rightStickY = rightStickY;
        this.yButton = yButton;
        this.bButton = bButton;
        this.aButton = aButton;
        this.xButton = xButton;
        this.crawlTrigger = crawlTrigger;
        this.slowRotate = slowRotate;
        this.dPad = dPad;
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
        ChassisSpeeds speeds = calculateSpeeds();
        handleToggleButtons();
        SmartDashboard.putBoolean("Field Oriented", fieldOriented);
        swerveSubsystem.setSpeed(speeds, fieldOriented);
    
    }
    private void handleToggleButtons() {
        if ((dPad.get() == 0) && fieldOrientedCache == false) {
            fieldOriented = !fieldOriented;
        }
        fieldOrientedCache = dPad.get() == 0;

        if ((dPad.get() == 90) && pointedModeCache == false) {
            pointedTurning = !pointedTurning;
        }
        pointedModeCache = dPad.get() == 90;
    }
    private ChassisSpeeds calculateSpeeds() {
        // X, Y, and Rotational speeds respecively.
        double rotationSpeed;
        if (isTurnButtonPressed()) {
            rotationSpeed = calculateTurningSpeedHotkey();
        }
        else if (pointedTurning) {
            rotationSpeed = calculateTurningSpeedPointed(rightStickX.get(), rightStickY.get());
        }
        else {
            rotationSpeed = calculateTurningSpeedsNormal(rightStickX.get());
        }
        double[] xySpeeds = calculateTranslationSpeeds(xSpdFunction.get(), ySpdFunction.get());
        return new ChassisSpeeds(xySpeeds[0], xySpeeds[1], rotationSpeed);
    }
    private double calculateTurningSpeedsNormal(double turningSpeed) {
        // Apply the deadzone. This will prevent the robot from moving at very small values
        turningSpeed = Math.abs(turningSpeed) > CONTROLLER_DEADZONE ? turningSpeed : 0.0;

        double turningSpeedMultiplier;
        if (crawlTrigger.get() >= 0.5) {
            turningSpeedMultiplier = SwerveDriveConstants.teleOpSlowAngularSpeed;
        } else {
            turningSpeedMultiplier = SwerveDriveConstants.teleOpMaxAngularSpeed;
        }
        if (slowRotate.get() >= 0.5 || crawlTrigger.get() >= 0.5) {
            turningSpeedMultiplier = SwerveDriveConstants.teleOpSlowAngularSpeed;
        } else {
            turningSpeedMultiplier = SwerveDriveConstants.teleOpMaxAngularSpeed;
        }
        turningSpeed = turningLimiter.calculate(turningSpeed);
        turningSpeed *= turningSpeedMultiplier;
        return turningSpeed;
    }
    private double calculateTurningSpeedPointed(double rx, double ry) {
        double angle = Math.atan2(rx, ry);
        angle = ((angle+2*Math.PI) % (Math.PI*2));
        angle *= (180/Math.PI);
        rotationController.setSetpoint(angle);
        return rotationController.calculate(swerveSubsystem.getHeading());
    }
    private double calculateTurningSpeedHotkey() {
        int angle;
        if (yButton.get()) {
            angle = 0;
        }
        else if (bButton.get()) {
            angle = 90;
        }
        else if (aButton.get()) {
            angle = 180;
        }
        else if (xButton.get()) {
            angle = 270;
        } else return 0;
        rotationController.setSetpoint(angle);
        return rotationController.calculate(swerveSubsystem.getHeading());
    }
    private double[] calculateTranslationSpeeds(double xSpeed, double ySpeed) {
        double[] xySpeeds = new double[2];
        double speed;
        if (crawlTrigger.get() >= 0.5) {
            speed = SwerveDriveConstants.TELEOP_SLOW_SPEED_METERS_PER_SECOND;
        } else {
            speed = SwerveDriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;
        }
        if (slowRotate.get() >= 0.5 || crawlTrigger.get() >= 0.5) {
        } else {
        }
        
        double magnitude = Math.hypot(ySpeed, xSpeed);
        // Apply the deadzone. This will prevent the robot from moving at very small values
        magnitude = Math.abs(magnitude) > CONTROLLER_DEADZONE ? magnitude : 0.0;
        magnitude = Math.pow(magnitude, 3);


        magnitude = Math.min(magnitude, 1);
        // Limit the acceleration for moving and rotation using the rate limiters
        // Using the rate limited value from 0 to 1, this will make a value of 1 move the robot at the configured maximum speed
        magnitude = xLimiter.calculate(magnitude);
        
        
        magnitude *= speed;
        xSpeed *= magnitude;
        ySpeed *= magnitude;
        
        xySpeeds[0] = xSpeed;
        xySpeeds[1] = ySpeed;

        return xySpeeds;
    }
    private boolean isTurnButtonPressed() {
        return aButton.get() || bButton.get() || yButton.get() || xButton.get();
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