/* 

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

public class Climber extends SubsystemBase {
    private final SparkFlex climberMotor1 = new SparkFlex(14, MotorType.kBrushless);
    private final SparkFlex climberMotor2 = new SparkFlex(15, MotorType.kBrushless);
    private final AbsoluteEncoder climber1Encoder = climberMotor1.getAbsoluteEncoder();
    private final AbsoluteEncoder climber2Encoder = climberMotor2.getAbsoluteEncoder();

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(4, 1);
    private final ProfiledPIDController controller1 =
            new ProfiledPIDController(1.3, 0, 0.0, constraints);
    private final ProfiledPIDController controller2 =
            new ProfiledPIDController(1.3, 0, 0.0, constraints);

    private double currentPosition1 = 0.0;
    private double currentVelocity1 = 0.0;
    private double currentPosition2 = 0.0;
    private double currentVelocity2 = 0.0;
    private double speed1 = 0.0;
    private double speed2 = 0.0;
    double gff = 0.7;

    public Climber() {
        // Set inverts.
        SparkFlexConfig config1 = new SparkFlexConfig();
        SparkFlexConfig config2 = new SparkFlexConfig();

        config1.inverted(false);
        config1.absoluteEncoder.inverted(false);
        config2.inverted(true);
        config2.absoluteEncoder.inverted(true);

        climberMotor1.configure(config1, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        climberMotor2.configure(config2, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command setSpeed(double speed) {
        return runOnce(() -> {
            speed1 = speed;
            speed2 = speed;
        });
    }

    public Command setPosition(double position) {
        return startRun(() -> {
            controller1.setGoal(position);
            controller2.setGoal(position);
        }, () -> {
        }).until(() -> {
            return controller1.atGoal() && controller2.atGoal();
        });
    }

    @Override
    public void periodic() {
        currentPosition1 = climber1Encoder.getPosition();
        currentVelocity1 = climber1Encoder.getVelocity();
        currentPosition2 = climber2Encoder.getPosition();
        currentVelocity2 = climber2Encoder.getVelocity();

        // speed1 = controller1.calculate(currentPosition1);
        // speed2 = controller2.calculate(currentPosition2);
        
        SmartDashboard.putNumber("Current Position1", currentPosition1);
        SmartDashboard.putNumber("Current Velocity1", currentVelocity1);
        SmartDashboard.putNumber("Current Position2", currentPosition2);
        SmartDashboard.putNumber("Current Velocity2", currentVelocity2);
        SmartDashboard.putNumber("Speed", speed1);
        SmartDashboard.putNumber("Speed", speed2);

        safeSet();
    }

    public void safeSet() {
        // Limit power.
        if (speed1 < -0.2)
            speed1 = -0.2;
        if (speed1 > +0.2)
            speed1 = +0.2;
        if (speed2 < -0.2)
            speed2 = -0.2;
        if (speed2 > +0.2)
            speed2 = +0.2;

        // System.out.println("climber " + speed1 + "/" + speed2 + ", " + currentPosition1 + "/" +currentPosition2);

        // Limit range.
        if (speed1 < 0.0 && currentPosition1 <= +0.18)
            speed1 = 0.0;
        if (speed1 > 0.0 && currentPosition1 >= +0.51)
            speed1 = 0.0;
        if (speed2 < 0.0 && currentPosition2 <= +0.15)
            speed2 = 0.0;
        if (speed2 > 0.0 && currentPosition2 >= +0.51)
            speed2 = 0.0;

        climberMotor1.set(speed1);
        climberMotor2.set(speed2);
    }
}
*/