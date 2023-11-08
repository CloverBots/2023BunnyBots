package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RabbitIntakeSubsystem;

public class RabbitIntakeCommand extends CommandBase{
    private final RabbitIntakeSubsystem rabbitIntakeSubsystem;
    private DoubleSupplier inSpeed;
    private DoubleSupplier outSpeed;

    public RabbitIntakeCommand(RabbitIntakeSubsystem rabbitIntakeSubsystem, DoubleSupplier inSpeed, DoubleSupplier outSpeed) {
        this.rabbitIntakeSubsystem = rabbitIntakeSubsystem;
        this.inSpeed = inSpeed;
        this.outSpeed = outSpeed;
    }

    @Override
    public void execute() {
        double in = inSpeed.getAsDouble();
        double out = outSpeed.getAsDouble();
        double speed = 0;

        if (in > 0.05 && out < 0.05) {
            speed = in;
        } else if (in < 0.05 && out > 0.05) {
            speed = -out;
        } else {
            speed = 0;
        }

        rabbitIntakeSubsystem.startIntake(speed);
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
