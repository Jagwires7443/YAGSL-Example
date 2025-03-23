package frc.robot.subsystems.swervedrive.subsystems;
import edu.wpi.first.wpilibj2.command.Command;

public class RunIntakeCommand extends Command {
    private final Intake intake;
    private final double speed;
    private final double duration;
    private long startTime;

    public RunIntakeCommand(Intake intake, double speed, double duration) {
        this.intake = intake;
        this.speed = speed;
        this.duration = duration * 1000; // Convert seconds to milliseconds
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        intake.setSpeed(speed); // Start the motors
    }

    @Override
    public void execute() {
        // Keep running the motors (no additional logic needed here)
    }

    @Override
    public boolean isFinished() {
        // Stop after the specified duration
        return System.currentTimeMillis() - startTime >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMotors(); // Stop the motors
    }
}