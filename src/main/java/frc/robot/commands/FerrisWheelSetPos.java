// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FerrisWheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FerrisWheelSetPos extends Command {
  /** Creates a new FerrisWheelVertical. */
  private final FerrisWheel ferrisWheel;

  private final double targetPosition;
  private static final double POSITION_TOLERANCE = .030; // Adjust tolerance as needed

  public FerrisWheelSetPos(FerrisWheel ferrisWheel, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ferrisWheel = ferrisWheel;
    this.targetPosition = targetPosition;
    addRequirements(ferrisWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ferrisWheel.setposition(targetPosition);
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
