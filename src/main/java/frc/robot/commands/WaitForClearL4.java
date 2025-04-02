// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitForClearL4 extends Command {
  private final DigitalInput sensor;

  /**
   * Command to wait for a game piece detection or timeout.
   *
   * @param sensor The DigitalInput sensor to check.
   * @param timeout The time in seconds before the command auto-exits.
   */
  public WaitForClearL4(DigitalInput sensor) {
    this.sensor = sensor; // Use the existing instance from FerrisWheel
  }

  @Override
  public void initialize() {}

  @Override
  public boolean isFinished() {
    return sensor.get();
  }

  @Override
  public void end(boolean interrupted) {

    // Print final sensor state when the command ends
    System.out.println("Game Piece Stuck if True " + sensor.get());
  }
}
