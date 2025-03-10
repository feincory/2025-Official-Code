// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.Elevator.m_elevatorlowered;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FerrisWheel extends SubsystemBase {
  /** Creates a new FerrisWheel. */
  private final TalonFX m_FerrisWheel = new TalonFX(20, "CANIVORE");

  TalonSRX m_coral = new TalonSRX(26);
  TalonSRX m_Algae = new TalonSRX(25);

  private final CANcoder m_cc = new CANcoder(35, "CANIVORE");

  private final StatusSignal<Angle> cc_pos = m_cc.getPosition();
  private final StatusSignal<AngularVelocity> cc_vel = m_cc.getVelocity();

  private final DutyCycleOut m_ferrisDutyCycleOut = new DutyCycleOut(0);
  private final PositionDutyCycle m_PositionDutyCycle = new PositionDutyCycle(0);
  static boolean m_fwheelclear;
  static double ferriswheelpos;
  static double coralplacepositionvalue;
  static double coralretreivepositionvalue;
  static double coralstartpositionvalue;

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
    fx_cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .05;
    fx_cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    fx_cfg.Slot0.kS = 0;
    fx_cfg.Slot0.kV = 0;
    fx_cfg.Slot0.kA = 0;
    fx_cfg.Slot0.kP = 50;
    fx_cfg.Slot0.kD = 1.5;
    fx_cfg.Slot0.kI = kfIc;
    fx_cfg.Slot0.kG = kfGc;
    fx_cfg.Voltage.PeakForwardVoltage = 12;
    fx_cfg.Voltage.PeakReverseVoltage = -12;
    // current limiting
    fx_cfg.CurrentLimits.SupplyCurrentLimit = 25;
    fx_cfg.CurrentLimits.SupplyCurrentLowerLimit = 30;
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    fx_cfg.CurrentLimits.StatorCurrentLimit = 30;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    // Configure MotionMagicExpo settings
    var motionMagicConfigs = fx_cfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // was 2
    motionMagicConfigs.MotionMagicExpo_kV = 2; // was 6
    motionMagicConfigs.MotionMagicExpo_kA = 2.25; // 4

    m_FerrisWheel.getConfigurator().apply(fx_cfg);
    m_FerrisWheel.setNeutralMode(NeutralModeValue.Coast);

    // constant values
    coralplacepositionvalue = .168;
    coralretreivepositionvalue = .624;
    coralstartpositionvalue = .524;

    createDashboards();
  }

  @Override
  public void periodic() {
    if (cc_pos.getValueAsDouble() > .51 && cc_pos.getValueAsDouble() < .525) {
      m_fwheelclear = true;
    } else {
      m_fwheelclear = false;
    }

    getpostion();
    outputferris();
    // ferriswheelpos = m_cc.getPosition();

  }

  // public StatusSignal<Angle> getmotoroutSignal() {
  //   return m_FerrisWheel.getBridgeOutput();
  // }

  public void placeposition() {
    if (m_elevatorlowered == false) {}
    m_FerrisWheel.setControl(m_PositionDutyCycle.withPosition(coralplacepositionvalue));
  }

  public void startingposition() {
    if (m_elevatorlowered == false) {}
    m_FerrisWheel.setControl(m_PositionDutyCycle.withPosition(coralstartpositionvalue));
  }

  public void setposition(double position) {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    m_FerrisWheel.setControl(m_request.withPosition(position));
    StatusSignal<Double> ferrisoutput = m_FerrisWheel.getDutyCycle();
    if (ferrisoutput.getValue() > 0) { // ferris wheel moves CC from climb side
      m_Algae.set(ControlMode.PercentOutput, ((Math.abs(ferrisoutput.getValue()) * -.5) - .2));

    } else {
      // ferris wheel moves CCW from climb side
      m_Algae.set(ControlMode.PercentOutput, ((Math.abs(ferrisoutput.getValue()) * -.3) - .2));
      m_coral.set(ControlMode.PercentOutput, (ferrisoutput.getValue() * -1.25));
    }
  }

  public void retreiveposition() {
    if (m_elevatorlowered == false) {}
    m_FerrisWheel.setControl(m_PositionDutyCycle.withPosition(coralretreivepositionvalue));
  }

  public void algaeposition() {
    if (m_elevatorlowered == false) {}
    m_FerrisWheel.setControl(m_PositionDutyCycle.withPosition(.911));
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

  public double getpostion() {
    return m_cc.getPosition().getValueAsDouble();
  }

  public boolean outputferris() {
    return m_fwheelclear;
  }

  public void createDashboards() {
    ShuffleboardTab ferristab = Shuffleboard.getTab("Ferris Wheel");
    Shuffleboard.getTab("Ferris Wheel").add("position", getpostion());
    Shuffleboard.getTab("Ferris Wheel").add("Ferris Wheel Clear", outputferris());
  }

  // Coral Intake
  public void coralin() {
    m_coral.set(ControlMode.PercentOutput, .6);
  }

  public void coralstop() {
    m_coral.set(ControlMode.PercentOutput, 0);
  }

  public void coralout() {
    m_coral.set(ControlMode.PercentOutput, -.7);
  }

  public void coraloutslow() {
    m_coral.set(ControlMode.PercentOutput, -.325);
  }

  public void coralhold() {
    m_coral.set(ControlMode.PercentOutput, .1);
  }

  // Algae Intake
  public void algaein() {
    m_Algae.set(ControlMode.PercentOutput, -.75);
  }

  public void algaehold() {
    m_Algae.set(ControlMode.PercentOutput, -.2);
  }

  public void algaestop() {
    m_Algae.set(ControlMode.PercentOutput, 0);
  }

  public void algaeout() {
    m_Algae.set(ControlMode.PercentOutput, 1);
  }
}
