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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

// Elevator Homing Command Speed
// import com.ctre.phoenix6.controls.PositionVoltage;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX m_liftlead = new TalonFX(18, "CANIVORE");

  private final TalonFX m_liftfollow = new TalonFX(19, "CANIVORE");

  private final DutyCycleOut m_liftoutput = new DutyCycleOut(0);
  private final PositionDutyCycle m_liftpPositionDutyCycle = new PositionDutyCycle(0);

  static boolean crashparallel;
  static boolean elevatorfirst;
  static boolean m_elevatorlowered;

  // private final double kGearRatio = 5.45;
  // private final double kinchesperrotation = 4;
  // private double kinchtorotation;
  // private double elevatorpos;

  // private static final double EXPONENT = 4.0; // Controls motion smoothing

  public Elevator() {
    TalonFXConfiguration elevator_cfg = new TalonFXConfiguration();
    elevator_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevator_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .3;
    elevator_cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .1;
    // pid config
    elevator_cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    elevator_cfg.Slot0.kS = 0;
    elevator_cfg.Slot0.kV = 0;
    elevator_cfg.Slot0.kA = 0;
    elevator_cfg.Slot0.kP = 1.5; // was 2
    elevator_cfg.Slot0.kI = 0;
    elevator_cfg.Slot0.kD = 0.1;
    elevator_cfg.Slot0.kG = 0;
    elevator_cfg.Voltage.PeakForwardVoltage = 12;
    elevator_cfg.Voltage.PeakReverseVoltage = -12;

    // current limiting
    elevator_cfg.CurrentLimits.SupplyCurrentLimit = 50;
    elevator_cfg.CurrentLimits.SupplyCurrentLowerLimit = 60;
    elevator_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevator_cfg.CurrentLimits.StatorCurrentLimit = 60;
    elevator_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    // Configure MotionMagicExpo settings
    var motionMagicConfigs = elevator_cfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0;
    motionMagicConfigs.MotionMagicExpo_kV = .07;
    motionMagicConfigs.MotionMagicExpo_kA = .075;

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

    m_elevatorlowered = true;
    crashparallel = false;
    elevatorfirst = false;
    // kinchtorotation = kGearRatio * kinchesperrotation;
    // elevatorpos = 5;

    createDashboards();
  }

  @Override
  public void periodic() {
    // elevatorpos = m_liftlead.getPosition();
    // This method will be called once per scheduler run
    // BooleanSupplier sup = () -> elevatorcrashposition;
  }

  // Elevator Clear Command
  public void setelevatorclear(double position) {
    m_liftlead.setControl(m_liftpPositionDutyCycle.withPosition(position));
  }

  // Elevator HomeLiftCommand
  public void setElevatorPower(double power) {
    m_liftlead.set(power);
  }

  // Elevator set position
  public void setElevatorpositon(double position) {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    // m_liftlead.setControl(m_liftpPositionDutyCycle.withPosition(position));
    m_liftlead.setControl(m_request.withPosition(position));
  }

  public double getpostion() {
    // return m_liftpPositionDutyCycle.getPositionMeasure().in(Rotation);
    return m_liftlead.getPosition().getValueAsDouble();
  }

  public void resetEncoder() {
    m_liftlead.setPosition(0);
  }

  ////////////////////////////////////////////////////////
  public void manualelevatorup() {
    m_liftlead.setControl(m_liftoutput.withOutput(-.2));
  }

  public void manualelevatordown() {
    m_liftlead.setControl(m_liftoutput.withOutput(.1));
  }

  public void setelevatorL4() {
    m_liftlead.setControl(m_liftoutput.withOutput(.25));
  }

  public void setelevatorL1() {
    m_liftlead.setControl(m_liftoutput.withOutput(-.25));
  }

  public void stopelevator() {
    m_liftlead.setControl(m_liftoutput.withOutput(-.025));
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

  public void crashparalleltrue() {
    crashparallel = true;
  }

  public void crashparallelfalse() {
    crashparallel = false;
  }

  public void elevatorfirsttrue() {
    elevatorfirst = true;
  }

  public void elevatorfirstfalse() {
    elevatorfirst = false;
  }

  public BooleanSupplier crashparallel() {
    return () -> crashparallel;
  }

  public BooleanSupplier elevatorfirst() {
    return () -> elevatorfirst;
  }

  public void createDashboards() {
    // ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    // Shuffleboard.getTab("ELEVATOR").add("TEST", getLeftPos());
    // Shuffleboard.getTab("Elevator").add("position", m_liftlead.getPosition());
    // elevatortab.addNumber("arm comp", this::armcompvalue)
    // .withSize(1,1)
    // .withPosition(0,0);
  }
}
