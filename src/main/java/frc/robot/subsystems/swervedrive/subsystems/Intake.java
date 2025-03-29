// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkFlex vortex1 = new SparkFlex(12, MotorType.kBrushless);
  private final SparkFlex vortex2 = new SparkFlex(13, MotorType.kBrushless);
  private final DigitalInput beamBreakSensor = new DigitalInput(5);

  public Intake() {
    // Set inverts.
    SparkFlexConfig config1 = new SparkFlexConfig();
    SparkFlexConfig config2 = new SparkFlexConfig();

    config1.inverted(false);
    config2.inverted(true);

    vortex1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    vortex2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  public void setSpeed(double speed) {
    vortex1.set(-speed);
    vortex2.set(speed);
  }

  public void stopMotors() {
    vortex1.set(0);
    vortex2.set(0);
  }

  public Command setIntakeSpeed(double speed) {
    return this.run(() -> setSpeed(speed));
  }

  // Method to check if the beam is broken
  public boolean isBeamBroken() {
    return !beamBreakSensor.get(); // Assuming 'false' means that the beam is broken
  }
}
