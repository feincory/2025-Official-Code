package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FerrisWheel;
import frc.robot.util.TransitionMatrix;

public class MoveToPositionCommand extends SequentialCommandGroup {
  public MoveToPositionCommand(
      Elevator elevator, FerrisWheel ferrisWheel, int currentKey, int targetKey) {
    // Check for an intermediate position requirement
    if (TransitionMatrix.requiresIntermediate(targetKey)) {
      int intermediate = TransitionMatrix.getIntermediatePosition(targetKey);
      addCommands(new MoveToPositionCommand(elevator, ferrisWheel, currentKey, intermediate));
      currentKey = intermediate; // Update current position before final move
    }

    // Run exit sequence if required
    if (TransitionMatrix.hasExitSequence(currentKey)) {
      addCommands(TransitionMatrix.getExitSequence(currentKey, elevator, ferrisWheel));
    }

    // Determine movement type
    TransitionMatrix.MovementType movementType =
        TransitionMatrix.getTransitionType(currentKey, targetKey);
    if (movementType == TransitionMatrix.MovementType.PARALLEL) {
      addCommands(
          new ParallelCommandGroup(
              new Elevatorsetpos(elevator, TransitionMatrix.getElevatorPosition(targetKey)),
              new FerrisWheelSetPos(
                  ferrisWheel, TransitionMatrix.getFerrisWheelPosition(targetKey))));
    } else if (movementType == TransitionMatrix.MovementType.SEQUENTIAL_ELEVATOR_FIRST) {
      addCommands(
          new Elevatorsetpos(elevator, TransitionMatrix.getElevatorPosition(targetKey)),
          new FerrisWheelSetPos(ferrisWheel, TransitionMatrix.getFerrisWheelPosition(targetKey)));
    } else if (movementType == TransitionMatrix.MovementType.SEQUENTIAL_FERRIS_FIRST) {
      addCommands(
          new FerrisWheelSetPos(ferrisWheel, TransitionMatrix.getFerrisWheelPosition(targetKey)),
          new Elevatorsetpos(elevator, TransitionMatrix.getElevatorPosition(targetKey)));
    }

    // Run entry sequence if required
    if (TransitionMatrix.hasEntrySequence(targetKey)) {
      addCommands(TransitionMatrix.getEntrySequence(targetKey, elevator, ferrisWheel));
    }
  }
}
