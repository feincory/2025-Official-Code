// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  TalonSRX m_coral = new TalonSRX(25);
 

  public CoralIntake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void coralin() {
    m_coral.set(ControlMode.PercentOutput,.20);
  }

  public void coralstop() {
 m_coral.set(ControlMode.PercentOutput,0);  }

  public void coralout() {
 m_coral.set(ControlMode.PercentOutput,-.20);  }
}
