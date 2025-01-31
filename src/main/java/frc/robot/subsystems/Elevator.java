// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.*;
// import static frc.robot.Constants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX m_liftlead = new TalonFX(18);

  private final TalonFX m_liftfollow = new TalonFX(19);

  private final DutyCycleOut m_liftoutput = new DutyCycleOut(0);
  private final PositionDutyCycle m_liftpPositionDutyCycle = new PositionDutyCycle(0);

  // private final double kGearRatio = 5.45;

  public Elevator() {
    TalonFXConfiguration elevator_cfg = new TalonFXConfiguration();
    elevator_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevator_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .3;
    elevator_cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1;
    // pid config
    elevator_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    elevator_cfg.Slot0.kP = .25;
    elevator_cfg.Slot0.kD = 0;
    elevator_cfg.Slot0.kI = 0;
    elevator_cfg.Slot0.kG = 0;
    elevator_cfg.Voltage.PeakForwardVoltage = 12;
    elevator_cfg.Voltage.PeakReverseVoltage = -12;
    m_liftlead.getConfigurator().apply(elevator_cfg);
    m_liftfollow.getConfigurator().apply(elevator_cfg);
    m_liftlead.setNeutralMode(NeutralModeValue.Brake);
    m_liftfollow.setNeutralMode(NeutralModeValue.Brake);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_liftlead.getConfigurator().apply(elevator_cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // initliftTalonFX(m_liftlead.getConfigurator());
    // initliftTalonFX(m_liftfollow.getConfigurator());

    /* Set followers to follow leader */
    m_liftfollow.setControl(new Follower(m_liftlead.getDeviceID(), false));

    /* Make sure all critical signals are synchronized */
    /*
     * Setting all these signals to 100hz means they get sent at the same time if
     * they're all on a CANivore
     */
    BaseStatusSignal.setUpdateFrequencyForAll(100, m_liftlead.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setelevatorL4() {
    m_liftlead.setControl(m_liftoutput.withOutput(.25));
  }

  public void setelevatorL1() {
    m_liftlead.setControl(m_liftoutput.withOutput(-.25));
  }

  public void stopelevator() {
    m_liftlead.setControl(m_liftoutput.withOutput(0));
  }

  public void setelevatorpos() {
    m_liftlead.setControl(m_liftpPositionDutyCycle.withPosition(10));
  }

  public void setelevatorposback() {
    m_liftlead.setControl(m_liftpPositionDutyCycle.withPosition(-10));
  }

  public StatusSignal<Angle> getLeftPos() {
    return m_liftlead.getPosition();
  }

  /**
   * Initialize a left drive TalonFX device from the configurator object
   *
   * @param cfg Configurator of the TalonFX device
   */
  private void initliftTalonFX(TalonFXConfigurator cfg) {
    var toApply = new TalonFXConfiguration();

    /*
     * User can change configs if they want, or leave this blank for factory-default
     */
    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    cfg.apply(toApply);

    /* And initialize position to 0 */
    cfg.setPosition(0);
  }
}
