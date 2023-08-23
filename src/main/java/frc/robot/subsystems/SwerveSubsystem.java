package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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
        SmartDashboard.putBoolean("reset gyro", false);
        SmartDashboard.putBoolean("resync turn encoders", false);
        this.modules = new SwerveModule[4];
        this.states = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };
        // Initializes each swerve module
        for (int i=0; i<4; i++) {
            modules[i] = new SwerveModule(SwerveModuleConfigurations.values()[i]);
        }
    }
    public void onEnable() {
        SmartDashboard.putBoolean("reset gyro", false);
        SmartDashboard.putBoolean("resync turn encoders", false);
        resetTurnEncoders();
    }
    public void zeroHeading() {
        gyro.reset();
    }
    public double getHeading() {
        // Because the NavX gives headings from -180 to 180 degrees, we need to convert it to a range of 0 to 360 degrees.
        // negative because we need CCW = positive
        return Math.IEEEremainder(-gyro.getAngle(), 360);
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
    public void resetOdometry() {
        odometer.resetPosition(getRotation2d(), getModulePositions(), new Pose2d());
    }
    public void setSpeed(double vx, double vy, double omegaRadsPerSecond, boolean fieldOriented) {
        // Construct a ChassisSpeeds object, which will contain the movement and rotation speeds that we want our robot to do.
        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            // Driving will be relative to field.
            // If this is enabled, then pressing forward will always move the robot forward, no matter its rotation.
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    vx, vy, omegaRadsPerSecond, getRotation2d());
        } else {
            // Driving will be relative to the robot.
            // If this is enabled, then pressing forward will move the robot in the direction that it is currently facing.
            chassisSpeeds = new ChassisSpeeds(vx, vy, omegaRadsPerSecond);
        }

        // This will take the speeds that we want our robot to move and turn at, and calculate the required direction and speed for each swerve module on the robot.
        SwerveModuleState[] moduleStates = SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Set the swerve modules to their required states.
        setModuleStates(moduleStates);
    }

    @Override
    public void periodic() {
        // Updates the odometer with the current rotation and distance travelled on each module.
        odometer.update(getRotation2d(), getModulePositions());
        for (int i = 0; i<4; i++) {
            modules[i].setDesiredState(states[i]);
        }
        SmartDashboard.putNumber("Gyro Heading", getHeading());
        //SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
        //SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());

        SmartDashboard.putString("Odometer Robot Location", getPose().getTranslation().toString());

        if (SmartDashboard.getBoolean("reset gyro", false)) {
            SmartDashboard.putBoolean("reset gyro", false);
            zeroHeading();
        }
        if (SmartDashboard.getBoolean("resync turn encoders", false)){
            SmartDashboard.putBoolean("resync turn encoders", false);
            resetTurnEncoders();
        }
        // Monitor absolute encoder values for configuration
        for (int i=0; i<modules.length; i++) {
            SmartDashboard.putNumber("abs "+SwerveDriveConstants.SwerveModuleConfigurations.values()[i].name(), modules[i].getAbsolutePosition());
        }
        // Monitor encoder values for configuration
        for (int i=0; i<modules.length; i++) {
            SmartDashboard.putNumber(SwerveDriveConstants.SwerveModuleConfigurations.values()[i].name(), Units.radiansToDegrees(modules[i].getTurningPosition()));
        }
        // for (int i=0; i<modules.length; i++) {
        //     SmartDashboard.putNumber("talon "+SwerveDriveConstants.SwerveModuleConfigurations.values()[i].name(), Units.radiansToDegrees(modules[i].getDriveMotorPosition()));
        // }
        
    }
    public void resetTurnEncoders() {
        for (SwerveModule module : modules) {
            module.resetTurnEncoder();
        }
    }
    public void resetDriveEncoders() {
        for (SwerveModule module : modules) {
            module.resetDriveEncoder();
        }
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