package frc.robot.subsystems.swervedrive.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public class HoldAndReturnFlipperCommand extends Command {
    private final Flipper flipper;
    private final double targetPosition;
    private double originalPosition;

    public HoldAndReturnFlipperCommand(Flipper flipper, double targetPosition) {
        this.flipper = flipper;
        this.targetPosition = targetPosition;
        addRequirements(flipper); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        // Save the original position of the flipper
       
        flipper.setFlipperPosition(targetPosition);
        System.out.println("Moving flipper to: " + targetPosition);
        
    }

    @Override
    public void execute() {
        // Optionally, monitor the flipper's position or add logic here
    }

    
    @Override
    public boolean isFinished() {
        // Optionally, add logic to determine when the command should finish
        return false; // Run until interrupted
    }
}