// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.*;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.google.flatbuffers.DoubleVector;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake */
  private final TalonFX m_tiltmtr = new TalonFX(37, "CANIVORE");
  private final TalonFX m_spinner = new TalonFX(38, "CANIVORE");
  private final CANcoder m_cc = new CANcoder(36, "CANIVORE");


  // private final StatusSignal<Angle> cc_pos = m_cc.getPosition();
  // private final StatusSignal<AngularVelocity> cc_vel = m_cc.getVelocity();

  private final DutyCycleOut m_ferrisDutyCycleOut = new DutyCycleOut(0);
  private final PositionDutyCycle m_PositionDutyCycle = new PositionDutyCycle(0);
  static double stowposition;
  static double l1ScorePosition;
  static double pickupposition;

  public GroundIntake() {




    /* Configure CANcoder to zero the magnet appropriately */
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1));
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(0));
    m_cc.getConfigurator().apply(cc_cfg);

    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackRemoteSensorID = m_cc.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
    fx_cfg.Feedback.RotorToSensorRatio = 50;
    fx_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .5;
    fx_cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .05;
    fx_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    fx_cfg.Slot0.kS = 0;
    fx_cfg.Slot0.kV = 0;
    fx_cfg.Slot0.kA = 0;
    fx_cfg.Slot0.kP = 10;
    fx_cfg.Slot0.kD = 0;
    fx_cfg.Slot0.kI = kfIc;
    fx_cfg.Slot0.kG = kfGc;
    fx_cfg.Voltage.PeakForwardVoltage = 12;
    fx_cfg.Voltage.PeakReverseVoltage = -12;

    fx_cfg.CurrentLimits.SupplyCurrentLimit = 25;
    fx_cfg.CurrentLimits.SupplyCurrentLowerLimit = 30;
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    fx_cfg.CurrentLimits.StatorCurrentLimit = 30;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    // Configure MotionMagicExpo settings
    var motionMagicConfigs = fx_cfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // was 2
    motionMagicConfigs.MotionMagicExpo_kV = 6; // was 6
    motionMagicConfigs.MotionMagicExpo_kA = 2.5; // 2.25



    //spinner configs
    TalonFXConfiguration spinner_cfg = new TalonFXConfiguration();
    spinner_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .5;
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
    stowposition = .14;
    pickupposition = -.14;
    l1ScorePosition = 0;



  }

  @Override
  public void periodic() {

  }

  // public StatusSignal<Angle> getmotoroutSignal() {
  //   return m_FerrisWheel.getBridgeOutput();
  // }





  public void stowposition() {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    m_tiltmtr.setControl(m_request.withPosition(stowposition));
  }

  public void pickupposition() {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    m_tiltmtr.setControl(m_request.withPosition(pickupposition));
  }

  public void l1ScorePosition() {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    m_tiltmtr.setControl(m_request.withPosition(l1ScorePosition));
  }




  public void setposition(double position) {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    m_tiltmtr.setControl(m_request.withPosition(position));
  }

  public void retreiveposition() {

    m_tiltmtr.setControl(m_PositionDutyCycle.withPosition(coralretreivepositionvalue));
  }

  public void algaeposition() {
    m_tiltmtr.setControl(m_PositionDutyCycle.withPosition(.911));
  }

  public void manferrisCW() {
    m_tiltmtr.setControl(m_ferrisDutyCycleOut.withOutput(-.20));
  }

  public void manferrisCCW() {
    m_tiltmtr.setControl(m_ferrisDutyCycleOut.withOutput(0.20));
  }

  public void ferrisstop() {
    m_tiltmtr.setControl(m_ferrisDutyCycleOut.withOutput(0));
  }

  public double getpostion() {
    return m_cc.getPosition().getValueAsDouble();
  }

  public boolean outputferris() {
    return m_fwheelclear;
  }




}
