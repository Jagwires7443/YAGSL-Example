
package frc.robot.subsystems.swervedrive.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.SparkBase;

public class Flipper extends SubsystemBase {

    private final SparkFlex flipperMotor;
    private final AbsoluteEncoder flipperEncoder;

    public double getFlipperPosition() {
    return flipperEncoder.getPosition();
    }

    public Flipper() {
        flipperMotor = new SparkFlex(9, MotorType.kBrushless);
        flipperEncoder = flipperMotor.getAbsoluteEncoder();
  
    
      }


    // Constants for arm control
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kPositionConversionFactor = 1.0;

    public static final double POSITION_STOW = 0.0;
    public static final double POSITION_CORAL = 50.0; // Placeholder, needs configuring!

    public void periodic() {
        // Get the current position and velocity from the encoder
        double currentPosition = flipperEncoder.getPosition();
        double currentVelocity = flipperEncoder.getVelocity();
    
        // Display the current position and velocity on the SmartDashboard
        SmartDashboard.putNumber("Current Position", currentPosition);
        SmartDashboard.putNumber("Current Velocity", currentVelocity);
    }
    public Flipper(int motorID) {
        //Initialize the Sparkflex motor
        flipperMotor = new SparkFlex (motorID, MotorType.kBrushless);
   
    //Initialize the encoder
    flipperEncoder = flipperMotor.getAbsoluteEncoder();

    //Optionally configure the encoder
    EncoderConfig encoderConfig = new EncoderConfig()
    .positionConversionFactor(kPositionConversionFactor * 360.0);} // Example: 1 rotation = 360 degrees
    
//Method to move the flipper to a specific position
public Command setFlipperPosition(double position){
    flipperMotor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
    return this.run(() -> setFlipperPosition(position));
}

public Command stopArm() {
    flipperMotor.stopMotor();
    return new InstantCommand(() -> flipperMotor.stopMotor(), this);
}
public Command POSITION_STOW() {
    return setFlipperPosition(POSITION_STOW);
}
public Command POSITION_CORAL() {
    return setFlipperPosition(POSITION_CORAL);
}
}

