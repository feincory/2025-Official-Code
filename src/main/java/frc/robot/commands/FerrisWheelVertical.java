// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FerrisWheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FerrisWheelVertical extends Command {
  /** Creates a new FerrisWheelVertical. */
  private final FerrisWheel ferrisWheel;

  // private double currentposition;

  private final double targetPosition = .524;
  private static final double POSITION_TOLERANCE = .015; // Adjust tolerance as needed

  public FerrisWheelVertical(FerrisWheel ferrisWheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ferrisWheel = ferrisWheel;
    addRequirements(ferrisWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ferrisWheel.startingposition();
    // double currentposition = ferrisWheel.getpostion();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ferris wheel vertical");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentposition = ferrisWheel.getpostion();
    return Math.abs(currentposition - targetPosition) < POSITION_TOLERANCE;
    // return false;
  }
}
