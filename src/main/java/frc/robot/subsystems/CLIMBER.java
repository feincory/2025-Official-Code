// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CLIMBER extends SubsystemBase {
  /** Creates a new CLIMBER. */
  private final TalonFX m_climber = new TalonFX(23, "CANIVORE");

  private final DutyCycleOut m_climbOutput = new DutyCycleOut(0);
  TalonFXConfiguration climb_cfg = new TalonFXConfiguration();

  public CLIMBER() {
    climb_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .3;
    m_climber.getConfigurator().apply(climb_cfg);
    m_climber.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbdown() {
    m_climber.setControl(m_climbOutput.withOutput(-.750));
  }

  public void climbup() {
    m_climber.setControl(m_climbOutput.withOutput(0.50));
  }

  public void climbstop() {
    m_climber.setControl(m_climbOutput.withOutput(.0));
  }
}
