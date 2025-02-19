package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FerrisWheel;
import frc.robot.util.TransitionMatrix;
import java.util.ArrayList;
import java.util.List;

public class MoveToPositionCommand extends SequentialCommandGroup {
  public MoveToPositionCommand(
      Elevator elevator, FerrisWheel ferrisWheel, int currentKey, int targetKey) {
    List<Command> commandList = new ArrayList<>();
    System.out.println("MoveToPositionCommand: " + currentKey + " -> " + targetKey);

    // 1️⃣ Exit sequence must run first and fully complete
    Command exitSequence = TransitionMatrix.getExitSequence(currentKey, elevator, ferrisWheel);
    if (exitSequence != null && !TransitionMatrix.requiresIntermediate(targetKey)) {
      commandList.add(new SequentialCommandGroup(exitSequence));
    }

    // 2️⃣ If an intermediate position is required, move there first
    if (TransitionMatrix.requiresIntermediate(targetKey)) {
      int intermediate = TransitionMatrix.getIntermediatePosition(targetKey);
      commandList.add(
          new Elevatorsetpos(elevator, TransitionMatrix.getElevatorPosition(intermediate)));
      commandList.add(
          new FerrisWheelSetPos(
              ferrisWheel, TransitionMatrix.getFerrisWheelPosition(intermediate)));
      currentKey = intermediate; // Update position
    }

    // 3️⃣ Main movement execution
    TransitionMatrix.MovementType movementType =
        TransitionMatrix.getTransitionType(currentKey, targetKey);
    if (movementType == TransitionMatrix.MovementType.SEQUENTIAL_ELEVATOR_FIRST) {
      commandList.add(
          new Elevatorsetpos(elevator, TransitionMatrix.getElevatorPosition(targetKey)));
      commandList.add(
          new FerrisWheelSetPos(ferrisWheel, TransitionMatrix.getFerrisWheelPosition(targetKey)));
    } else if (movementType == TransitionMatrix.MovementType.SEQUENTIAL_FERRIS_FIRST) {
      commandList.add(
          new SequentialCommandGroup( // ✅ Force Ferris Wheel first
              new FerrisWheelSetPos(
                  ferrisWheel, TransitionMatrix.getFerrisWheelPosition(targetKey))));
      commandList.add(
          new Elevatorsetpos(elevator, TransitionMatrix.getElevatorPosition(targetKey)));
    } else {
      commandList.add(
          new Elevatorsetpos(elevator, TransitionMatrix.getElevatorPosition(targetKey)));
      commandList.add(
          new FerrisWheelSetPos(ferrisWheel, TransitionMatrix.getFerrisWheelPosition(targetKey)));
    }

    // 4️⃣ Entry sequence must run last
    Command entrySequence = TransitionMatrix.getEntrySequence(targetKey, elevator, ferrisWheel);
    if (entrySequence != null) {
      commandList.add(new SequentialCommandGroup(entrySequence)); // ✅ Enforce sequential execution
    }

    // 🔹 Print the command list to verify order
    System.out.println("MoveToPositionCommand Execution Order:");
    for (Command cmd : commandList) {
      System.out.println(" - " + cmd.getClass().getSimpleName());
    }

    // 5️⃣ Add all commands to the sequential command group
    addCommands(commandList.toArray(new Command[0]));
  }
}
