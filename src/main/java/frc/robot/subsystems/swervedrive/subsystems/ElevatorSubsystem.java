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
import edu.wpi.first.wpilibj.XboxController;

public class ElevatorSubsystem extends SubsystemBase {
    private final XboxController driverXbox = new XboxController(0); // Initialize XboxController on port 0
    private final SparkFlex elevatorMotor = new SparkFlex(10, MotorType.kBrushless);
    private final SparkFlex elevatorFollower = new SparkFlex(11, MotorType.kBrushless);
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
    private final SparkLimitSwitch upperLimit = elevatorMotor.getForwardLimitSwitch();
    private final SparkLimitSwitch lowerLimit = elevatorMotor.getReverseLimitSwitch();

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(15.0, 15.0);
    private final ProfiledPIDController controller =
            new ProfiledPIDController(2.0, 0, 0.1, constraints);

    private double currentPosition = 0.0;
    private double currentVelocity = 0.0;
    private double speed = 0.0;
    private boolean atLower = false;
    private boolean atUpper = false;
     double gff = -0.2; // Add a small negative feedforward for downward motion
   
 
     public static final double POSITION_INTAKE = 108.0; 
     public static final double POSITION_L1 = 0.0; //Use elastic and check elevator motor rotations
     public static final double POSITION_L2 = 0.0;
     public static final double POSITION_L3 = 0.0;

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
        return this.runOnce(() -> {
            this.speed = speed;
        });
        }
            
    public Command setPosition(double position) {
        return run(() -> {
            controller.setGoal(position); // Set the target position for the PID controller
            this.speed = controller.calculate(currentPosition); // Calculate PID output
            safeSet(); // Apply the speed safely
        }).until(controller::atGoal);
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
        SmartDashboard.putNumber("Elevator Current Position", currentPosition);
     

        safeSet();
    }

    public void safeSet() {
        // Limit power.
        if (speed < -0.95)
            speed = -0.95;
        if (speed > +0.95)
            speed = +0.95;

        // System.out.println("Elevator " + atLower + ", " + atUpper + ", " + speed + ", " + currentPosition);

        //Limit range.
        //if (speed < 0.0 && lowerLimit.isPressed())
           //speed = 0.0;
       //if (speed > 0.0 && upperLimit.isPressed())
           //speed = 0.0;

        elevatorMotor.set(speed);
    }

    public Command manualMoveUp(double speed) {
        return run(() -> {
            if (!atUpper) { // Check the upper limit
                this.speed = speed; // Set the speed to move up
            } else {
                this.speed = 0.0; // Stop if the upper limit is reached
            }
            safeSet();
        });
    }
    
    public Command manualMoveDown(double speed) {
        return run(() -> {
            if (!atLower) { // Check the lower limit
                this.speed = -speed; // Set the speed to move down
            } else {
                this.speed = 0.0; // Stop if the lower limit is reached
            }
            safeSet();
        }
        );
    }
    

    public Command stopElevator() {
        return runOnce(() -> {
            this.speed = 0.0; // Stop the elevator
            safeSet();

    
        });
    }
}


