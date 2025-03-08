// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralGround extends SubsystemBase {
  /** Creates a new CoralGround. */
  private SparkMax motor;

  private SparkMax spinner;

  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  private boolean coralgroundhomed;

  private DigitalInput m_proxswitch;

  public CoralGround() {

    m_proxswitch = new DigitalInput(1);
    motor = new SparkMax(41, MotorType.kBrushless);
    spinner = new SparkMax(40, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();

    encoder = motor.getEncoder();
    motorConfig = new SparkMaxConfig();
    motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    coralgroundhomed = true;
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(1.75) // WAS 2
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(2 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motorConfig
        .closedLoop
        .maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(5000) // WAS 2500
        .maxAcceleration(3000) // WAS 2500
        .allowedClosedLoopError(.25)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(.25, ClosedLoopSlot.kSlot1);

    motor.configure(motorConfig, null, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void homingroutine() {
    if (!m_proxswitch.get() == true) {
      coralgroundhomed = true;
      encoder.setPosition(0);
      System.out.print("homed homie");
      closedLoopController.setReference(0 - .001, ControlType.kDutyCycle);
    } else {
      closedLoopController.setReference(-.10, ControlType.kDutyCycle);
    } // This method will be called once per scheduler run
    spinner.set(0);
  }

  public void resetEncoder() {
    // This method will be called once per scheduler run
    encoder.setPosition(0);
  }

  public void initstoragepos() {
    // This method will be called once per scheduler run
    closedLoopController.setReference(
        .25, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    spinner.set(0);
  }

  public void storagepos() {
    // This method will be called once per scheduler run
    if (coralgroundhomed) {
      closedLoopController.setReference(
          .25, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
    spinner.set(0);
  }

  public void shootpos() {
    // This method will be called once per scheduler run
    if (coralgroundhomed) {
      closedLoopController.setReference(
          3.5, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      spinner.set(0);
    }
  }

  public void pickupos() {
    // This method will be called once per scheduler run
    if (coralgroundhomed) {
      closedLoopController.setReference(
          15, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      spinner.set(.6);
    }
  }

  public void runspinner() {
    // This method will be called once per scheduler run
    spinner.set(-.65);
  }

  public void stopspinner() {
    // This method will be called once per scheduler run
    spinner.set(.0);
  }

  public void stop() {
    // This method will be called once per scheduler run
    closedLoopController.setReference(0, ControlType.kDutyCycle);
  }
}
