package frc.robot.util;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FerrisWheel;
import java.util.HashMap;
import java.util.Map;

public class TransitionMatrix {
  public enum MovementType {
    PARALLEL,
    SEQUENTIAL_ELEVATOR_FIRST,
    SEQUENTIAL_FERRIS_FIRST
  }

  private static final Map<Integer, Double> elevatorPositions = new HashMap<>();
  private static final Map<Integer, Double> ferrisWheelPositions = new HashMap<>();
  private static final Map<String, MovementType> movementRules = new HashMap<>();
  private static final Map<Integer, Integer> intermediatePositions = new HashMap<>();

  static {
    // Define elevator positions for each index
    elevatorPositions.put(0, elevatorstow);
    elevatorPositions.put(1, elevatorRetreivepos);
    elevatorPositions.put(2, elevatorL2pos);
    elevatorPositions.put(3, elevatorL3pos);
    elevatorPositions.put(4, elevatorL4pos);
    elevatorPositions.put(5, elevatorRetreiveAlgaepos);
    elevatorPositions.put(6, elevatorL2Algaepos);
    elevatorPositions.put(7, elevatorL3Algaepos);
    elevatorPositions.put(8, elevatorprocessor);
    elevatorPositions.put(9, elevatornetpos);
    elevatorPositions.put(10, elevatorclimb);
    elevatorPositions.put(11, elevatoridlepos);

    // Define ferris wheel positions for each index
    ferrisWheelPositions.put(0, ferriswheelvert);
    ferrisWheelPositions.put(1, ferriscoralretreive);
    ferrisWheelPositions.put(2, ferriscoralplace);
    ferrisWheelPositions.put(3, ferriscoralplace);
    ferrisWheelPositions.put(4, ferriscoralplace);
    ferrisWheelPositions.put(5, ferrisalageretreive);
    ferrisWheelPositions.put(6, ferrisAlgaeReefPic);
    ferrisWheelPositions.put(7, ferrisAlgaeReefPic);
    ferrisWheelPositions.put(8, ferrisalgaeprocessor);
    ferrisWheelPositions.put(9, ferrisAlgaeNet);
    ferrisWheelPositions.put(10, ferriswheelvert);
    ferrisWheelPositions.put(11, ferriswheelvert);

    // Define movement types
    movementRules.put("0-1", MovementType.SEQUENTIAL_ELEVATOR_FIRST);

    movementRules.put("1-11", MovementType.SEQUENTIAL_FERRIS_FIRST);
    movementRules.put("11-2", MovementType.SEQUENTIAL_FERRIS_FIRST);

    movementRules.put("2-11", MovementType.SEQUENTIAL_ELEVATOR_FIRST);
    movementRules.put("11-1", MovementType.SEQUENTIAL_ELEVATOR_FIRST);

    movementRules.put("1-11", MovementType.SEQUENTIAL_FERRIS_FIRST);
    movementRules.put("11-3", MovementType.SEQUENTIAL_FERRIS_FIRST);

    movementRules.put("3-11", MovementType.SEQUENTIAL_ELEVATOR_FIRST);
    movementRules.put("11-1", MovementType.SEQUENTIAL_ELEVATOR_FIRST);

    movementRules.put("1-4", MovementType.PARALLEL);
    movementRules.put("4-1", MovementType.PARALLEL);
    movementRules.put("4-1", MovementType.PARALLEL);

    // If going to position 2, first move to position 2

    intermediatePositions.put(1, 11);
    intermediatePositions.put(2, 11);
    intermediatePositions.put(3, 11);
  }

  public static double getElevatorPosition(int key) {
    return elevatorPositions.getOrDefault(key, 0.0);
  }

  public static double getFerrisWheelPosition(int key) {
    return ferrisWheelPositions.getOrDefault(key, ferriswheelvert);
  }

  public static MovementType getTransitionType(int currentKey, int targetKey) {
    return movementRules.getOrDefault(currentKey + "-" + targetKey, MovementType.PARALLEL);
  }

  public static boolean requiresIntermediate(int targetKey) {
    return intermediatePositions.containsKey(targetKey);
  }

  public static int getIntermediatePosition(int targetKey) {
    return intermediatePositions.getOrDefault(targetKey, targetKey);
  }

  // EXIT SEQUENCES: Reverse of entry sequence for safely leaving a position
  public static Command getExitSequence(int key, Elevator elevator, FerrisWheel ferrisWheel) {
    Double elevatorTarget = getElevatorPosition(key);
    Double ferrisWheelTarget = getFerrisWheelPosition(key);
    System.out.println("Exit sequence for " + key);
    if (key == 3) { // Example: Leaving position 4 requires reversing the movements
      return new SequentialCommandGroup(
          new FerrisWheelSetPos(ferrisWheel, ferriswheelvert) // Reverse rotation
          // new Elevatorsetpos(elevator, elevatorTarget) // Lower elevator back to previous state
          );
    }
    if (key == 1) {
      // System.out.print("Exiting position 1");
      return new SequentialCommandGroup(
          new FerrisWheelSetPos(ferrisWheel, ferriswheelvert), // Reverse rotation
          new Elevatorsetpos(elevator, elevatorTarget) // Lower elevator back to previous state
          );
    }
    if (key == 2) { // Example: Moving into position 4 requires special steps
      return new SequentialCommandGroup(
          new Elevatorsetpos(elevator, elevatorTarget), // Move elevator to safe position first
          new FerrisWheelSetPos(ferrisWheel, ferrisWheelTarget) // Rotate to halfway position
          );
    }

    return null; // No special exit sequence required
  }
  // ENTRY SEQUENCES: Define movements needed to safely reach a target position
  public static Command getEntrySequence(int key, Elevator elevator, FerrisWheel ferrisWheel) {
    Double elevatorTarget = getElevatorPosition(key);
    Double ferrisWheelTarget = getFerrisWheelPosition(key);

    if (key == 1) {
      // System.out.print("Entry for 1");
      return new SequentialCommandGroup(
          // new Elevatorsetpos(elevator, elevatorTarget), // Move elevator to safe position first
          new FerrisWheelSetPos(ferrisWheel, ferrisWheelTarget) // Rotate to halfway position
          );
    }
    if (key == 2) { // Example: Moving into position 4 requires special steps
      return new SequentialCommandGroup(
          new FerrisWheelSetPos(ferrisWheel, ferrisWheelTarget), // Rotate to halfway position
          new Elevatorsetpos(elevator, elevatorTarget) // Move elevator to safe position first
          );
    }

    return null; // No special entry sequence required
  }
}
