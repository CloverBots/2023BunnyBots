package frc.robot.subsystems;

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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.SwerveDriveConstants.SwerveModuleConfiguration;

import static frc.robot.SwerveDriveConstants.SwerveModules;

//**Need updated imports for the motors, encoders, and math inputs


public class SwerveModule {


    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;
    
    private final CANCoder turningEncoder;

    private SwerveModuleConfiguration config;

    public SwerveModule(SwerveModules moduleInfo) {
        this.config = moduleInfo.config;

        driveMotor = new TalonFX(config.driveMotorID);
        turningMotor = new CANSparkMax(config.turnMotorID, MotorType.kBrushless);
        configureCANCoder(turningEncoder);
        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec); if not using drive encoder

        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec); //Update for Encoder type

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }
    
    //public double getDrivePosition() {
      //  return driveEncoder.getPosition();
   // }

    public double getTurningPosition() {
        return turningEncoder.getAbsolutePosition();
    }

   // public double getDriveVelocity() {
    //    return driveEncoder.getVelocity();
    //}

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
    }

    public double getRPM() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getSelectedSensorVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turningMotor.set(0);
    }

    private void configureCANCoder(CANCoder cancoder) {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = 0;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoder.configAllSettings(config);
    }

}