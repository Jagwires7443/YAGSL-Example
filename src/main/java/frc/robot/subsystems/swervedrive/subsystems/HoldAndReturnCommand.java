package frc.robot.subsystems.swervedrive.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public class HoldAndReturnCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double targetPosition;

    public HoldAndReturnCommand(ElevatorSubsystem elevator, double position) {
        this.elevator = elevator;
        this.targetPosition = position;
        addRequirements(elevator); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        elevator.setPosition(targetPosition); // Move to the target position
        System.out.println("Moving elevator to position: " + targetPosition);
    }

    @Override
    public void execute() {
        // Continuously hold the position (handled by the subsystem's PID controller)
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("HoldAndReturnCommand interrupted.");
        } else {
            System.out.println("HoldAndReturnCommand completed.");
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}