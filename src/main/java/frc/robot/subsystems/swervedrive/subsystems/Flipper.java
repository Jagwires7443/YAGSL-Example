package frc.robot.subsystems.swervedrive.subsystems;

import com.revrobotics.AbsoluteEncoder;
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

public class Flipper extends SubsystemBase {
    private final SparkFlex flipperMotor = new SparkFlex(9, MotorType.kBrushless);
    private final AbsoluteEncoder flipperEncoder = flipperMotor.getAbsoluteEncoder();

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(68, 48);
    private final ProfiledPIDController controller =
            new ProfiledPIDController(1.55, 0, 0, constraints);

    private double currentPosition = 0.0;
    private double currentVelocity = 0.0;
    private double speed = 0.0;

    public static final double POSITION_CORAL = 0.81; 
    public static final double POSITION_STOW = 0.18;
    public static final double POSITION_L4 = 0.70;  //unused
    public static final double POSITION_BARGE = 0.95; //unused

    public Flipper() {
        // Set inverts.
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(false);
        config.absoluteEncoder.inverted(false);

        flipperMotor.configure(config, ResetMode.kResetSafeParameters,
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
        currentPosition = flipperEncoder.getPosition();
        currentVelocity = flipperEncoder.getVelocity();

        speed = controller.calculate(currentPosition);

        SmartDashboard.putNumber("Current Position1", currentPosition);
        SmartDashboard.putNumber("Current Velocity1", currentVelocity);
        SmartDashboard.putNumber("Speed", speed);

        safeSet();
    }

    public void safeSet() {
        // Limit power.
        if (speed < -1.0)
            speed = -1.0;
        if (speed > +1.0)
            speed = +1.0;

        double appliedSpeed = speed;

        // Limit range.
        if (speed < 0.0 && currentPosition <= +0.18)
            appliedSpeed = 0.0;
        if (speed > 0.0 && currentPosition >= +0.75)
            appliedSpeed = 0.0;

        // System.out.println("Flipper " + appliedSpeed + ", " + speed + ", " + currentPosition);

        flipperMotor.set(appliedSpeed);
    }

    public Command Coral() { return setPosition(POSITION_CORAL);}

    public Command Stow() { return setPosition(POSITION_STOW);}
}
