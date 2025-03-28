package frc.robot.subsystems.swervedrive.subsystems;


import edu.wpi.first.wpilibj2.command.Command;



public class HoldAndReturnCommand extends Command {
    private final ElevatorSubsystem elevator;
    private double targetHeight;

    public HoldAndReturnCommand(ElevatorSubsystem elevator, double height) {
        this.elevator = elevator;
        this.targetHeight = height;
    }

    @Override
    public void initialize() {
        elevator.setHeight(targetHeight);
        System.out.println("Moving elevator to: " + targetHeight);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}