package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RabbitIntakeSubsystem;

public class RabbitIntakeCommand extends CommandBase{
    private final RabbitIntakeSubsystem rabbitIntakeSubsystem;
    private final DoubleSupplier rightJoystickY;

    public RabbitIntakeCommand(RabbitIntakeSubsystem rabbitIntakeSubsystem, DoubleSupplier rightJoystickY) {
        this.rabbitIntakeSubsystem = rabbitIntakeSubsystem;
        this.rightJoystickY = rightJoystickY;
        addRequirements(rabbitIntakeSubsystem);
    }

    @Override
    public void execute() {
        double speed = -rightJoystickY.getAsDouble() * .5;

        if (Math.abs(speed) < 0.05) {
            speed = 0;
        }

        rabbitIntakeSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        rabbitIntakeSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
