// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CLIMBER extends SubsystemBase {
  /** Creates a new CLIMBER. */
  private final TalonFX m_climber = new TalonFX(23, "CANIVORE");

  private final DutyCycleOut m_climbOutput = new DutyCycleOut(0);
  TalonFXConfiguration climb_cfg = new TalonFXConfiguration();
  Servo m_funnelrelease;
  boolean funnelokaytorelease;

  // ⬅️ NEW: DigitalInput for the limit switch
  private final DigitalInput m_topLimitSwitch = new DigitalInput(3); // DIO port 0

  public CLIMBER() {
    climb_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .3;
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
    m_climber.setControl(m_climbOutput.withOutput(-1)); // was -.75
  }

  public void climbup() {
    if (m_topLimitSwitch.get()) {
      // Switch not pressed – OK to move up
      m_climber.setControl(m_climbOutput.withOutput(.9));
    } else {
      // Switch pressed – stop!
      m_climber.setControl(m_climbOutput.withOutput(0.0));
    }
    // m_climber.setControl(m_climbOutput.withOutput(.5)); // was .3
  }

  public void climbhold() {
    if (m_topLimitSwitch.get()) {
      // Switch not pressed – OK to move up
      m_climber.setControl(m_climbOutput.withOutput(.05));
    } else {
      // Switch pressed – stop!
      m_climber.setControl(m_climbOutput.withOutput(0.0));
    }

    // m_climber.setControl(m_climbOutput.withOutput(.05));
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
