package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class BallFeederSubsystem extends SubsystemBase{
    private final int CURRENT_LIMIT = 10;

    private final CANSparkMax motor = new CANSparkMax(IDs.FEEDER_DEVICE, MotorType.kBrushless);

    /** Creates a new FeederSubsystem. */
    public BallFeederSubsystem() {

        motor.setInverted(true);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

}
