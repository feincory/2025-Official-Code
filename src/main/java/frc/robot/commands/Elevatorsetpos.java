// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevatorsetpos extends Command {
  private Elevator lift;

  private static final double POSITION_TOLERANCE = 1;
  private final double elevatorpostion;
  /** Creates a new ElevatorL4. */
  public Elevatorsetpos(Elevator lift, double elevatorpos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lift = lift;
    this.elevatorpostion = elevatorpos;
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lift.setElevatorpositon(elevatorpostion);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.print("Lift at position");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentposition = lift.getpostion();
    // System.out.println(currentposition);
    return Math.abs(currentposition - elevatorpostion) < POSITION_TOLERANCE;
  }
}
