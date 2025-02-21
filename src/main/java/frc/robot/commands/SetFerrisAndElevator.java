// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.elevatoridlepos;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.FerrisWheel;
// import java.util.function.BooleanSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetFerrisAndElevator extends SequentialCommandGroup {
  /** Creates a new SetFerrisAndElevator. */
  boolean condition;

  public SetFerrisAndElevator(
      Elevator m_Elevator, double elevatortarget, FerrisWheel m_FerrisWheel, double ferristarget) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // BooleanSupplier conditiontest = () -> true;

    addCommands(
        new ConditionalCommand(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new Elevatorsetpos(m_Elevator, elevatoridlepos),
                    new FerrisWheelVertical(m_FerrisWheel),
                    new SequentialCommandGroup(
                        new Elevatorsetpos(m_Elevator, elevatortarget),
                        new FerrisWheelSetPos(m_FerrisWheel, ferristarget))),
                new SequentialCommandGroup(
                    new FerrisWheelVertical(m_FerrisWheel),
                    new Elevatorsetpos(m_Elevator, elevatoridlepos),
                    new SequentialCommandGroup(
                        new FerrisWheelSetPos(m_FerrisWheel, ferristarget),
                        new Elevatorsetpos(m_Elevator, elevatortarget))),

                // conditon for 2nd conditional
                m_Elevator.elevatorfirst()),

            // false true case
            new SequentialCommandGroup(
                new Elevatorsetpos(m_Elevator, elevatoridlepos),
                new FerrisWheelVertical(m_FerrisWheel),
                new FerrisWheelSetPos(m_FerrisWheel, ferristarget),
                new Elevatorsetpos(m_Elevator, elevatortarget)),
            // conditon for 1st conditional
            m_Elevator.crashparallel()));

    // addCommands(
    //   new ConditionalCommand(
    //   new ConditionalCommand(
    //     new SequentialCommandGroup(
    //           new FerrisWheelVertical(m_FerrisWheel),//true true command
    //           new Elevatorsetpos(m_Elevator, elevatoridlepos),//true true command
    //             new ParallelCommandGroup(//true true command
    //               new FerrisWheelSetPos(m_FerrisWheel, ferristarget),//true true command
    //               new Elevatorsetpos(m_Elevator, elevatortarget))), //true true command
    //     new SequentialCommandGroup(
    //           new Elevatorsetpos(m_Elevator, elevatoridlepos),//true false command
    //           new FerrisWheelVertical(m_FerrisWheel),//true false command
    //           new ParallelCommandGroup(//true false command
    //               new FerrisWheelSetPos(m_FerrisWheel, ferristarget),//true false command
    //               new Elevatorsetpos(m_Elevator, elevatortarget))), //true false command
    //     m_Elevator.crashonentrystatus()),

    //   new ParallelCommandGroup(
    //         new FerrisWheelVertical(m_FerrisWheel),
    //         new Elevatorsetpos(m_Elevator, elevatoridlepos),
    //           new SequentialCommandGroup(
    //             new FerrisWheelSetPos(m_FerrisWheel, ferristarget),
    //             new Elevatorsetpos(m_Elevator, elevatortarget))),
    //   new ParallelCommandGroup(
    //         new Elevatorsetpos(m_Elevator, elevatoridlepos),
    //         new FerrisWheelVertical(m_FerrisWheel),
    //         new SequentialCommandGroup(
    //             new FerrisWheelSetPos(m_FerrisWheel, ferristarget),
    //             new Elevatorsetpos(m_Elevator, elevatortarget))),
    //   m_Elevator.crashonexitstatus());

    /*
    case 1
    seq
    ferris
    elevator
    par
    ferris
    elevator

    case 2
    seq
    elevator
    ferris wheel
    par
    ferris
    elevator

    case 3
    par
    ferris
    elevator
    seq
    ferris
    elevator

    case 4
    par
    elevator
    ferris
    seq
    elevator
    ferris






    */
  }
}
