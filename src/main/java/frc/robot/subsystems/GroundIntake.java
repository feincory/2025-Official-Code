// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake */
  private final TalonFX m_tiltmtr = new TalonFX(37, "CANIVORE");

  private final TalonFX m_spinner = new TalonFX(38, "CANIVORE");
  private final CANcoder m_cc = new CANcoder(36, "CANIVORE");

  // private final StatusSignal<Angle> cc_pos = m_cc.getPosition();
  // private final StatusSignal<AngularVelocity> cc_vel = m_cc.getVelocity();

  private final DutyCycleOut m_spinnerdDutyCycleOut = new DutyCycleOut(0);
  private final PositionDutyCycle m_PositionDutyCycle = new PositionDutyCycle(0);
  static double stowposition;
  static double l1ScorePosition;
  static double pickupposition;

  public GroundIntake() {

    /* Configure CANcoder to zero the magnet appropriately */
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(.75));
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(-.15));
    m_cc.getConfigurator().apply(cc_cfg);

    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackRemoteSensorID = m_cc.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fx_cfg.Feedback.SensorToMechanismRatio = 1.0;

    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.Feedback.RotorToSensorRatio = 50;
    fx_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .5;
    fx_cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .05;

    fx_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    fx_cfg.Slot0.kS = 0;
    fx_cfg.Slot0.kV = 0;
    fx_cfg.Slot0.kA = 0;
    fx_cfg.Slot0.kP = 15;
    fx_cfg.Slot0.kD = 0;
    fx_cfg.Slot0.kI = 0;
    fx_cfg.Slot0.kG = .5;
    fx_cfg.Voltage.PeakForwardVoltage = 12;
    fx_cfg.Voltage.PeakReverseVoltage = -12;

    fx_cfg.CurrentLimits.SupplyCurrentLimit = 40;
    fx_cfg.CurrentLimits.SupplyCurrentLowerLimit = 40;
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    fx_cfg.CurrentLimits.StatorCurrentLimit = 40;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    // Configure MotionMagicExpo settings
    var motionMagicConfigs = fx_cfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // was 2
    motionMagicConfigs.MotionMagicExpo_kV = 5; // was 6
    motionMagicConfigs.MotionMagicExpo_kA = 5; // 2.25

    // spinner configs
    TalonFXConfiguration spinner_cfg = new TalonFXConfiguration();
    spinner_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .2;
    spinner_cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .05;
    spinner_cfg.Voltage.PeakForwardVoltage = 12;
    spinner_cfg.Voltage.PeakReverseVoltage = -12;

    // current limiting
    spinner_cfg.CurrentLimits.SupplyCurrentLimit = 25;
    spinner_cfg.CurrentLimits.SupplyCurrentLowerLimit = 30;
    spinner_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    spinner_cfg.CurrentLimits.StatorCurrentLimit = 30;
    spinner_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    m_tiltmtr.getConfigurator().apply(fx_cfg);
    m_tiltmtr.setNeutralMode(NeutralModeValue.Brake);

    m_spinner.getConfigurator().apply(spinner_cfg);
    m_spinner.setNeutralMode(NeutralModeValue.Coast);

    // constant values
    stowposition = .35;
    pickupposition = -.02; // was -.05
    l1ScorePosition = .31;

    // stowposition = .6;
    // pickupposition = .7;
    // l1ScorePosition = .65;
  }

  @Override
  public void periodic() {}

  public void stowposition() {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    m_tiltmtr.setControl(m_request.withPosition(stowposition));
    System.out.println("Target" + stowposition);
  }

  public void pickupposition() {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    m_tiltmtr.setControl(m_request.withPosition(pickupposition));
    System.out.println("Target" + pickupposition);
  }

  public void ClimbPosition() {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    m_tiltmtr.setControl(m_request.withPosition(.13));
  }

  public void l1ScorePosition() {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    m_tiltmtr.setControl(m_request.withPosition(l1ScorePosition));
  }

  public void spinnerfwd() {
    m_spinner.setControl(m_spinnerdDutyCycleOut.withOutput(-.25));
  }

  public void spinnerrev() {
    m_spinner.setControl(m_spinnerdDutyCycleOut.withOutput(0.70));
  }

  public void spinnerstop() {
    m_spinner.setControl(m_spinnerdDutyCycleOut.withOutput(0));
  }
}
