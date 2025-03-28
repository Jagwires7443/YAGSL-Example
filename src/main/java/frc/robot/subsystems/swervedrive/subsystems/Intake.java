// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive.subsystems;

//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  SparkFlex vortex1;
  SparkFlex vortex2;
  private final DigitalInput beamBreakSensor; //most likely can be removed
  public Intake() {
    vortex1 = new SparkFlex(12, MotorType.kBrushless);
    vortex2 = new SparkFlex(13, MotorType.kBrushless);
    //Initialize IR beam break sensor
    beamBreakSensor = new DigitalInput(5);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("vortex1 volts", vortex1.getAppliedOutput());
    SmartDashboard.putNumber("vortex1 current", vortex1.getOutputCurrent());
    
   // Display the beam break sensor value on the SmartDashboard
    SmartDashboard.putBoolean("Beam Break Sensor", isBeamBroken());
  }

  public void setSpeed(double speed) {
    vortex1.set(speed);
    vortex2.set(-speed);
    System.out.println(vortex1.getAppliedOutput());
  }
  public Command stopMotors() {
    return this.run(() -> {
      vortex1.set(0);
      vortex2.set(0);
    });
  }
  public Command setIntakeSpeed(double speed) {
    return this.run(() -> setSpeed(speed));
  }
  //Method to check if the beam is broken
  public boolean isBeamBroken(){
    return !beamBreakSensor.get(); //Assuming 'false' means that the beam is broken
 
}
}

