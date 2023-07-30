package frc.robot.subsystems;

import static frc.robot.constants.SwerveDriveConstants.SwerveModuleConfigurations;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.SwerveDriveConstants;

/**
 * Represents a Swerve Module on the robot. Contains movement and turning functionality for the wheel.
*/
public class SwerveModule {
    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;
    private final CANCoder turningEncoder;

    protected SwerveModuleConfigurations config;

    private PIDController turningPidController;

    /**
     * Constructs a new SwerveModule. 
     * @param config The configuration for this module. This is how the SwerveModule gets its constants (Motor IDs, offsets, etc.)
     */
    public SwerveModule(SwerveModuleConfigurations config) {
        this.config = config;
        this.driveMotor = new TalonFX(config.driveMotorID);
        this.turningMotor = new CANSparkMax(config.turnMotorID, MotorType.kBrushless);
        this.turningEncoder = new CANCoder(config.CANCoderID);
        configureCANCoder(turningEncoder);
        
        // This makes getPosition() on the turning motor encoder give it's rotation in radians.
        turningMotor.getEncoder().setPositionConversionFactor(SwerveDriveConstants.TURNING_ENCODER_TO_RAD);
        // This makes getVelocity() on the turning motor encoder give its velocity in radians/second.
        turningMotor.getEncoder().setVelocityConversionFactor(SwerveDriveConstants.TURNING_ENCODER_TO_RADS_PER_SECOND);

        driveMotor.setInverted(true);
        turningMotor.setInverted(true);
        turningPidController = new PIDController(
            SwerveDriveConstants.kPTurning,
            SwerveDriveConstants.kITurning,
            SwerveDriveConstants.kDTurning
        );
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        resetEncoders();
    }
    public double getTurningPosition() {
        return turningMotor.getEncoder().getPosition();
    }
    public double getAbsolutePosition() {
        return turningEncoder.getAbsolutePosition();
    }
    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }
    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.getEncoder().setPosition(Units.degreesToRadians(getAbsolutePosition()));
    }
    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * SwerveDriveConstants.DRIVE_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getSelectedSensorVelocity(), new Rotation2d(getTurningPosition()));
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getSelectedSensorPosition() * SwerveDriveConstants.DRIVE_ENCODER_TO_METERS, new Rotation2d(getTurningPosition()));
    }
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / SwerveDriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + config.name() + "] desired state", state.toString());
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turningMotor.set(0);
    }

    private void configureCANCoder(CANCoder cancoder) {
        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        encoderConfig.magnetOffsetDegrees = config.encoderOffset;
        encoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoder.configAllSettings(encoderConfig);
    }

}