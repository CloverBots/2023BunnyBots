package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RabbitDeploySubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RabbitDeployCommand extends CommandBase {

    private final RabbitDeploySubsystem rabbitDeploySubsystem;
    private double position;
    private int direction;
    private double speed;

    public RabbitDeployCommand(RabbitDeploySubsystem rabbitDeploySubsystem, double position, double speed) {
        this.rabbitDeploySubsystem = rabbitDeploySubsystem;
        this.position = position;
        this.speed = speed;
        // TO-DO limit position values
        addRequirements(rabbitDeploySubsystem);
    }

    @Override
    public void initialize() {
        direction = 1;

        if (rabbitDeploySubsystem.getEncoderPosition() > position) {
            direction = -1;
        }
        if (Math.abs(rabbitDeploySubsystem.getEncoderPosition() - position) < 3) {
            direction = 0;
        }
    }

    @Override
    public void execute() {
        rabbitDeploySubsystem.setDeploySpeed(speed * direction);
    }

    @Override
    public void end(boolean interrupted) {
        rabbitDeploySubsystem.setDeploySpeed(0);
        System.out.println(rabbitDeploySubsystem.getEncoderPosition());
    }

    @Override
    public boolean isFinished() {
        if (direction == -1 && rabbitDeploySubsystem.getEncoderPosition() <= position) {
            return true;
        } else if (direction == 1 && rabbitDeploySubsystem.getEncoderPosition() >= position) {
            return true;
        } else if (direction == 0) {
            return true;
        }

        return false;
    }
    
}
