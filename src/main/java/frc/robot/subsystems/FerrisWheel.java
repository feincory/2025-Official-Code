// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.Elevator.m_elevatorlowered;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FerrisWheel extends SubsystemBase {
  /** Creates a new FerrisWheel. */
  private final TalonFX m_FerrisWheel = new TalonFX(20, "CANIVORE");

  private final CANcoder m_cc = new CANcoder(35, "CANIVORE");

  private final StatusSignal<Angle> cc_pos = m_cc.getPosition();
  private final StatusSignal<AngularVelocity> cc_vel = m_cc.getVelocity();

  private final DutyCycleOut m_ferrisDutyCycleOut = new DutyCycleOut(0);
  private final PositionDutyCycle m_PositionDutyCycle = new PositionDutyCycle(0);
  static boolean m_fwheelclear;
  static double ferriswheelpos;
  static double coralplacepositionvalue;
  static double coralretreivepositionvalue;

  public FerrisWheel() {

    m_fwheelclear = false;

    /* Configure CANcoder to zero the magnet appropriately */
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1));
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(0));
    m_cc.getConfigurator().apply(cc_cfg);

    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackRemoteSensorID = m_cc.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
    fx_cfg.Feedback.RotorToSensorRatio = 83.740234375;
    fx_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .5;
    fx_cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .2;
    fx_cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    fx_cfg.Slot0.kP = kfPc;
    fx_cfg.Slot0.kD = kfDc;
    fx_cfg.Slot0.kI = kfIc;
    fx_cfg.Slot0.kG = kfGc;
    fx_cfg.Voltage.PeakForwardVoltage = 12;
    fx_cfg.Voltage.PeakReverseVoltage = -12;

    m_FerrisWheel.getConfigurator().apply(fx_cfg);
    m_FerrisWheel.setNeutralMode(NeutralModeValue.Coast);

    // constant values
    coralplacepositionvalue = -.25;
    coralretreivepositionvalue = .25;
  }

  @Override
  public void periodic() {
    // ferriswheelpos = m_cc.getPosition();

  }

  public void placeposition() {
    if (m_elevatorlowered == false) {}
    m_FerrisWheel.setControl(m_PositionDutyCycle.withPosition(coralplacepositionvalue));
  }

  public void retreiveposition() {
    if (m_elevatorlowered == false) {}
    m_FerrisWheel.setControl(m_PositionDutyCycle.withPosition(coralretreivepositionvalue));
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
