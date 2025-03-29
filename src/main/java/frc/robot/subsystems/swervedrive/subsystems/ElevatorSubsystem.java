package frc.robot.subsystems.swervedrive.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkFlex elevatorMotor = new SparkFlex(10, MotorType.kBrushless);
    private final SparkFlex elevatorFollower = new SparkFlex(11, MotorType.kBrushless);
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
    private final SparkLimitSwitch upperLimit = elevatorMotor.getForwardLimitSwitch();
    private final SparkLimitSwitch lowerLimit = elevatorMotor.getReverseLimitSwitch();

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(10.0, 10.0);
    private final ProfiledPIDController controller =
            new ProfiledPIDController(1.0, 0, 0, constraints);

    private double currentPosition = 0.0;
    private double currentVelocity = 0.0;
    private double speed = 0.0;
    private boolean atLower = false;
    private boolean atUpper = false;

    public ElevatorSubsystem() {
        // Set inverts.
        SparkFlexConfig config = new SparkFlexConfig();
        SparkFlexConfig followerConfig = new SparkFlexConfig();

        config.inverted(false);

        followerConfig.follow(elevatorMotor, true);

        elevatorMotor.configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        elevatorFollower.configure(followerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command setSpeed(double speed) {
        return runOnce(() -> {
            this.speed = speed;
        });
    }

    public Command setPosition(double position) {
        return startRun(() -> {
        controller.setGoal(position);
        }, () -> {
        }).until(() -> {
            return controller.atGoal();
        });
    }

    @Override
    public void periodic() {
        currentPosition = elevatorEncoder.getPosition();
        currentVelocity = elevatorEncoder.getVelocity();
        atLower = lowerLimit.isPressed();
        atUpper = upperLimit.isPressed();

        if (currentPosition != 0.0 && atLower)
            elevatorEncoder.setPosition(0.0);

        speed = controller.calculate(currentPosition);

        SmartDashboard.putNumber("Current Position1", currentPosition);
        SmartDashboard.putNumber("Current Velocity1", currentVelocity);
        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Elevator Motor Revolutions", currentPosition);

        safeSet();
    }

    public void safeSet() {
        // Limit power.
        if (speed < -0.02)
            speed = -0.02;
        if (speed > +1.0)
            speed = +1.0;

        // System.out.println("Elevator " + atLower + ", " + atUpper + ", " + speed + ", " + currentPosition);

        // Limit range.
        // if (speed < 0.0 && lowerLimit.isPressed())
        //     speed = 0.0;
        // if (speed > 0.0 && upperLimit.isPressed())
        //    speed = 0.0;

        elevatorMotor.set(speed);
    }
    
}
