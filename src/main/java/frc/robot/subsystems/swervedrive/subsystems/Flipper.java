package frc.robot.subsystems.swervedrive.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase;

public class Flipper extends SubsystemBase {

    private final SparkFlex flipperMotor;
    private final AbsoluteEncoder flipperEncoder;

     double kP = 2.0;
     double kI = 0;
     double k = 0.0;
     double gF = 10.0;
     double kV = 19.0;
     double kFF = 0.9;

    public double getFlipperPosition() {
    return flipperEncoder.getPosition();
    }

    public Flipper() {
        flipperMotor = new SparkFlex(9, MotorType.kBrushless);
        flipperEncoder = flipperMotor.getAbsoluteEncoder();
        SparkFlexConfig flipperConfig = new SparkFlexConfig();
   
        double currentPosition = flipperEncoder.getPosition();
        flipperMotor.getClosedLoopController().setReference(currentPosition, SparkBase.ControlType.kPosition);
        
      }

    public static final double POSITION_STOW = -10; //Placeholder, needs configuring!
    public static final double POSITION_CORAL = 30; // Placeholder, needs configuring!

    

    public void periodic() {
        // Get the current position and velocity from the encoder
        double currentPosition = flipperEncoder.getPosition();
        double currentVelocity = flipperEncoder.getVelocity();
    
        // Display the current position and velocity on the SmartDashboard
        SmartDashboard.putNumber("Current Position", currentPosition);
        SmartDashboard.putNumber("Current Velocity", currentVelocity);
        SmartDashboard.putNumber("Flipper Position", flipperEncoder.getPosition());
        SmartDashboard.putNumber("Flipper volts", flipperMotor.getAppliedOutput());
        SmartDashboard.putNumber("Flipper currents", flipperMotor.getOutputCurrent());
        SmartDashboard.putNumber("Flipper Motor Output", flipperMotor.getAppliedOutput());
        SmartDashboard.putNumber("Flipper Encoder Position", flipperEncoder.getPosition());
        
        double targetPosition = POSITION_CORAL; // Example: Use a predefined target position
     
        //if (currentPosition < targetPosition) {
           // flipperMotor.set(0.5); // Move up
       // } else {
            //flipperMotor.set(0); // Stop
       // }
            
    }
    public Flipper(int motorID) {
        //Initialize the Sparkflex motor
        flipperMotor = new SparkFlex (motorID, MotorType.kBrushless);
   
    //Initialize the encoder
    flipperEncoder = flipperMotor.getAbsoluteEncoder();


    }
    public double setFlipperPosition() {
        return flipperEncoder.getPosition() * 360;
    }
   
    
//Method to move the flipper to a specific position
public Command setFlipperPosition(double position) {
    return new InstantCommand(() -> {
        flipperMotor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
    }, this);
}

public Command stopArm() {
    return new InstantCommand(() -> flipperMotor.stopMotor(), this);
}
public Command POSITION_STOW() {
    return setFlipperPosition(POSITION_STOW);
}
public Command POSITION_CORAL() {
    return setFlipperPosition(POSITION_CORAL);
}
}
