// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FerrisWheel extends SubsystemBase {
  /** Creates a new FerrisWheel. */
  private final TalonFX m_FerrisWheel = new TalonFX(20, "CANIVORE");

  private final DutyCycleOut m_ferrisDutyCycleOut = new DutyCycleOut(0);

  public FerrisWheel() {
    m_FerrisWheel.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void manferrisCW() {
    m_FerrisWheel.setControl(m_ferrisDutyCycleOut.withOutput(-.20));
  }

  public void manferrisCCW() {
    m_FerrisWheel.setControl(m_ferrisDutyCycleOut.withOutput(0.20));
  }

  public void ferrisstop() {
    m_FerrisWheel.setControl(m_ferrisDutyCycleOut.withOutput(0));
  }
}
