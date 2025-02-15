// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.elevatoridlepos;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.FerrisWheel;
import java.util.function.BooleanSupplier;

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
    BooleanSupplier conditiontest = () -> true;
    // addCommands(
    //     new FerrisWheelVertical(m_FerrisWheel),
    //     new Elevatorsetpos(m_Elevator, elevatoridlepos),
    //     new ParallelCommandGroup(
    //         new Elevatorsetpos(m_Elevator, elevatortarget),
    //         new FerrisWheelSetPos(m_FerrisWheel, ferristarget)));

    // if (new InstantCommand()m_Elevator.elevatorstatus) {
    //   addCommands( // ontrue
    //       new SequentialCommandGroup(
    //           new Elevatorsetpos(m_Elevator, elevatoridlepos),
    //           new FerrisWheelVertical(m_FerrisWheel),
    //           new ParallelCommandGroup(
    //               new Elevatorsetpos(m_Elevator, elevatortarget),
    //               new FerrisWheelSetPos(m_FerrisWheel, ferristarget))));
    // } else {
    //   addCommands(
    //       new SequentialCommandGroup(
    //           new FerrisWheelVertical(m_FerrisWheel),
    //           new Elevatorsetpos(m_Elevator, elevatoridlepos),
    //           new ParallelCommandGroup(
    //               new Elevatorsetpos(m_Elevator, elevatortarget),
    //               new FerrisWheelSetPos(m_FerrisWheel, ferristarget))));
    //   addCommands(new InstantCommand(m_Elevator::elevatorstatus));
    // }
    addCommands(
        new ConditionalCommand(
            // ontrue
            new SequentialCommandGroup(
                new Elevatorsetpos(m_Elevator, elevatoridlepos),
                new FerrisWheelVertical(m_FerrisWheel),
                new ParallelCommandGroup(
                    new Elevatorsetpos(m_Elevator, elevatortarget),
                    new FerrisWheelSetPos(m_FerrisWheel, ferristarget))),

            // onfalse
            new SequentialCommandGroup(
                new FerrisWheelVertical(m_FerrisWheel),
                new Elevatorsetpos(m_Elevator, elevatoridlepos),
                new ParallelCommandGroup(
                    new Elevatorsetpos(m_Elevator, elevatortarget),
                    new FerrisWheelSetPos(m_FerrisWheel, ferristarget))),
            m_Elevator.elevatorcrashSupplier()));

    // addCommands(
    //     new ConditionalCommand(
    //         // ontrue
    //         new SequentialCommandGroup(
    //             new Elevatorsetpos(m_Elevator, elevatoridlepos),
    //             new FerrisWheelVertical(m_FerrisWheel),
    //             new ParallelCommandGroup(
    //                 new Elevatorsetpos(m_Elevator, elevatortarget),
    //                 new FerrisWheelSetPos(m_FerrisWheel, ferristarget))),

    //         // onfalse
    //         new SequentialCommandGroup(
    //             new FerrisWheelVertical(m_FerrisWheel),
    //             new Elevatorsetpos(m_Elevator, elevatoridlepos),
    //             new ParallelCommandGroup(
    //                 new Elevatorsetpos(m_Elevator, elevatortarget),
    //                 new FerrisWheelSetPos(m_FerrisWheel, ferristarget))),

    //         // condition
    //         conditiontest));
    //  //m_Elevator.elevatorcrashSupplier()));
  }
}
