// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitForClearL4 extends Command {
  private final DigitalInput sensor;
  private final Timer timer = new Timer();
  private final double timeout;

  /**
   * Command to wait for a game piece detection or timeout.
   *
   * @param sensor The DigitalInput sensor to check.
   * @param timeout The time in seconds before the command auto-exits.
   */
  public WaitForClearL4(DigitalInput sensor, double timeout) {
    this.sensor = sensor; // Use the existing instance from FerrisWheel
    this.timeout = timeout;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return sensor.get();
  }

  @Override
  public void end(boolean interrupted) {

    timer.stop();
    // Print final sensor state when the command ends
    System.out.println(
        "WaitForGamePieceCommand ended. Final sensor state: "
            + sensor.get()
            + ", Timeout reached: "
            + timer.hasElapsed(timeout));
  }
}
