package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.IDs;

public class RabbitDeploy {
    private final int CURRENT_LIMIT = 10;

    private final CANSparkMax motor = new CANSparkMax(IDs.RABBIT_DEPLOY_DEVICE, MotorType.kBrushless);

    public static final double LOWER_ENDPOINT = 0; 

    public static final double UPPER_ENDPOINT = 10; //Needs to be updated 

    public void RabbitDeploySubsystem() {
        motor.setSmartCurrentLimit(CURRENT_LIMIT);
    
        motor.setIdleMode(IdleMode.kBrake);
    
        motor.setInverted(false);
    }

    public void setDeploySpeed(double speed) {
        if ((getEncoderPosition() <= LOWER_ENDPOINT && speed < 0) ||
          (getEncoderPosition() >= UPPER_ENDPOINT && speed > 0)) {
        speed = 0;
        }
        motor.set(speed);
    }

    public double getEncoderPosition() {
        return motor.getEncoder().getPosition(); 
      }

    public void resetEncoder() {
        motor.getEncoder().setPosition(0);
    }   
}