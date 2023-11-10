package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallIntakeSubsystem;

public class BallIntakeCommand extends CommandBase {
    private final DoubleSupplier leftJoystickY;
    private final BallIntakeSubsystem ballIntakeSubsystem;

    public BallIntakeCommand(BallIntakeSubsystem ballIntakeSubsystem, DoubleSupplier leftJoystickY) {
        this.ballIntakeSubsystem = ballIntakeSubsystem;
        this.leftJoystickY = leftJoystickY;
        addRequirements(ballIntakeSubsystem);
    }

    @Override
    public void execute() {
        double speed = -leftJoystickY.getAsDouble() * .5;

        if (Math.abs(speed) < 0.05) {
            speed = 0;
        }

        ballIntakeSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        ballIntakeSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}