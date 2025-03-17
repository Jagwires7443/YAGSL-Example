// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive.subsystems;

//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  SparkFlex vortex1;
  SparkFlex vortex2;
  public Intake() {
    vortex1 = new SparkFlex(12, MotorType.kBrushless);
    vortex2 = new SparkFlex(13, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("vortex1 volts", vortex1.getAppliedOutput());
    SmartDashboard.putNumber("vortex1 current", vortex1.getOutputCurrent());
  }

  public void setSpeed(double speed) {
    vortex1.set(speed);
    vortex2.set(-speed);
    System.out.println(vortex1.getAppliedOutput());
  }

  public Command setIntakeSpeed(double speed) {
    return this.run(() -> setSpeed(speed));
  }
}
