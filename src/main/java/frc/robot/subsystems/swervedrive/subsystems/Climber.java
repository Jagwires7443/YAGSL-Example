package frc.robot.subsystems.swervedrive.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    SparkFlex climberMotor1;
    SparkFlex climberMotor2;

    //private final AbsoluteEncoder climber1Encoder;
    //private final AbsoluteEncoder climber2Encoder;
    
    public Climber() {

        climberMotor1 = new SparkFlex(14, MotorType.kBrushless);
        climberMotor2 = new SparkFlex(15, MotorType.kBrushless);
    }


    @Override
    public void periodic() {
        double currentPosition = climberMotor1.getEncoder().getPosition();
        double currentVelocity = climberMotor1.getEncoder().getVelocity();

        //Display
        SmartDashboard.putNumber("Current Position", currentPosition);
        SmartDashboard.putNumber("Current Velocity", currentVelocity);
    }

    public Command setSpeed(double speed) {
        climberMotor1.set(speed);
        climberMotor2.set(-speed);
        return this.run(() -> setSpeed(speed));
     
    }
    public Command ReverseClimber() {
        return this.run(() -> setSpeed(-1));
    }
    public void stopMotors() {
        climberMotor1.set(0);
        climberMotor2.set(0);
    }


    }
    
    

