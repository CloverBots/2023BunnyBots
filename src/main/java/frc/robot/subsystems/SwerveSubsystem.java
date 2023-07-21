package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;
import frc.robot.SwerveDriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    // The current *desired* states of all four modules.
    private SwerveModuleState[] states;

    private SwerveModule[] modules;

    // private final SwerveModule frontLeft = new SwerveModule(
    //         DriveConstants.kFrontLeftDriveMotorPort,
    //         DriveConstants.kFrontLeftTurningMotorPort,
    //         //DriveConstants.kFrontLeftDriveEncoderReversed, //need?
    //         DriveConstants.kFrontLeftTurningEncoderReversed,);

    private final AHRS gyro = new AHRS(IDs.AHRS_PORT_ID);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(SwerveDriveConstants.swerveKinematics,
            new Rotation2d(getHeading()));
    
    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getCurrentModuleStates());
        
        for (int i = 0; i<4; i++) {
            modules[i].setDesiredState(states[i]);
        }
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveModuleState[] getCurrentModuleStates() {
        return new SwerveModuleState[] {
            modules[0].getState(),
            modules[1].getState(),
            modules[2].getState(),
            modules[3].getState()
        };
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.maxSpeedMetersPerSecond);
        this.states = desiredStates;
    }
}