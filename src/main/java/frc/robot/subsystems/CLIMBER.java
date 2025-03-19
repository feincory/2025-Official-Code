// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CLIMBER extends SubsystemBase {
  /** Creates a new CLIMBER. */
  private final TalonFX m_climber = new TalonFX(23, "CANIVORE");

  private final DutyCycleOut m_climbOutput = new DutyCycleOut(0);
  TalonFXConfiguration climb_cfg = new TalonFXConfiguration();
  Servo m_funnelrelease;
  boolean funnelokaytorelease;

  public CLIMBER() {
    climb_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .4;
    m_climber.getConfigurator().apply(climb_cfg);
    m_climber.setNeutralMode(NeutralModeValue.Brake);
    m_funnelrelease = new Servo(0);
    funnelokaytorelease = false;
    m_funnelrelease.setAngle(20);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbdown() {
    m_climber.setControl(m_climbOutput.withOutput(-.6)); // was -.75
  }

  public void climbup() {
    m_climber.setControl(m_climbOutput.withOutput(.3)); // was .5
  }

  public void climbhold() {
    m_climber.setControl(m_climbOutput.withOutput(.05));
  }

  public void climbstop() {
    m_climber.setControl(m_climbOutput.withOutput(.0));
  }

  public void okaytorelease() {
    funnelokaytorelease = true;
  }

  public void funnelrelease() {
    if (funnelokaytorelease) {
      m_funnelrelease.setAngle(175);
    }
  }

  public void resetfunnel() {
    if (funnelokaytorelease) {
      m_funnelrelease.setAngle(20);
    }
  }
}
