package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;
import frc.robot.constants.SwerveDriveConstants;
import static frc.robot.constants.SwerveDriveConstants.SwerveModuleConfigurations;

public class SwerveSubsystem extends SubsystemBase {

    /** 
     * The current *desired* states of all four modules.
     */ 
    private SwerveModuleState[] states;

    /** Contains all 4 Swerve Modules */
    private SwerveModule[] modules;

    // private final SwerveModule frontLeft = new SwerveModule(
    //         DriveConstants.kFrontLeftDriveMotorPort,
    //         DriveConstants.kFrontLeftTurningMotorPort,
    //         //DriveConstants.kFrontLeftDriveEncoderReversed, //need?
    //         DriveConstants.kFrontLeftTurningEncoderReversed,);

    private final AHRS gyro = new AHRS(IDs.AHRS_PORT_ID);

    /** This will track the robot's X and Y position, as well as its rotation. */
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(SwerveDriveConstants.swerveKinematics, getRotation2d(), 
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        }
    );
    
    public SwerveSubsystem() {
        // The gyroscope takes a second to fully calibrate itself after initializing the object.
        // The Thread.sleep() makes sure that the gyroscope is properly calibrated before zeroing the heading.
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        // Initializes each swerve module
        for (int i=0; i<4; i++) {
            modules[i] = new SwerveModule(SwerveModuleConfigurations.values()[i]);
        }
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        // Because the NavX gives headings from -180 to 180 degrees, we need to convert it to a range of 0 to 360 degrees.
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        // Updates the odometer with the current rotation and distance travelled on each module.
        odometer.update(getRotation2d(), getModulePositions());
        for (int i = 0; i<4; i++) {
            modules[i].setDesiredState(states[i]);
        }
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
    
    public void stopModules() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        };
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        this.states = desiredStates;
    }
}