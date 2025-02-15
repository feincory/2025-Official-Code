// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  TalonSRX m_Algae = new TalonSRX(25);

  public AlgaeIntake() {}

  @Override
  public void periodic() {}

  public void algaein() {
    m_Algae.set(ControlMode.PercentOutput, .75);
  }

  public void algaestop() {
    m_Algae.set(ControlMode.PercentOutput, -.2);
  }

  public void algaeout() {
    m_Algae.set(ControlMode.PercentOutput, -1);
  }
  // This method will be called once per scheduler run
}
