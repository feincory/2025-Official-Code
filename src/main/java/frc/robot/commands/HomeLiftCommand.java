// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeLiftCommand extends Command {
  /** Creates a new HomeLiftCommand. */
  private final Elevator lift;

  private final DigitalInput homeSwitch;
  private static final double HOMING_SPEED = -0.2; // Slow descent
  private static final double TIMEOUT = 3.0; // Stop after 3 seconds

  private Timer timer = new Timer();

  public HomeLiftCommand(Elevator lift, int switchPort) {
    this.lift = lift;
    this.homeSwitch = new DigitalInput(switchPort);
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!homeSwitch.get()) { // Switch is triggered (active low)
      lift.setElevatorPower(0);
      lift.resetEncoder(); // Set position to zero
      timer.stop();
    } else {
      lift.setElevatorPower(HOMING_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift.setElevatorPower(0);
    if (timer.hasElapsed(TIMEOUT)) {
      System.out.println("ERROR: Lift homing failed! Sensor not detected.");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !homeSwitch.get() || timer.hasElapsed(TIMEOUT);
  }
}
