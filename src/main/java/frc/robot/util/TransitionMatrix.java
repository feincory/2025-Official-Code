package frc.robot.util;

import static frc.robot.Constants.elevatorL2Algaepos;
import static frc.robot.Constants.elevatorL2pos;
import static frc.robot.Constants.elevatorL3Algaepos;
import static frc.robot.Constants.elevatorL3pos;
import static frc.robot.Constants.elevatorL4pos;
import static frc.robot.Constants.elevatorRetreiveAlgaepos;
import static frc.robot.Constants.elevatorRetreivepos;
import static frc.robot.Constants.elevatorclimb;
import static frc.robot.Constants.elevatoridlepos;
import static frc.robot.Constants.elevatornetpos;
import static frc.robot.Constants.elevatorprocessor;
import static frc.robot.Constants.elevatorstow;
import static frc.robot.Constants.ferrisAlgaeNet;
import static frc.robot.Constants.ferrisAlgaeReefPic;
import static frc.robot.Constants.ferrisalageretreive;
import static frc.robot.Constants.ferrisalgaeprocessor;
import static frc.robot.Constants.ferriscoralplace;
import static frc.robot.Constants.ferriscoralretreive;
import static frc.robot.Constants.ferriswheelvert;

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
  private static final Map<Integer, Command> entrySequences = new HashMap<>();
  private static final Map<Integer, Command> exitSequences = new HashMap<>();
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

    // Define movement types
    movementRules.put("0-1", MovementType.PARALLEL);
    movementRules.put("1-2", MovementType.SEQUENTIAL_ELEVATOR_FIRST);
    movementRules.put("2-3", MovementType.SEQUENTIAL_FERRIS_FIRST);
    movementRules.put("3-4", MovementType.PARALLEL);

    // Define entry sequences (special movements before reaching target)
    entrySequences.put(
        4,
        new SequentialCommandGroup(
            new Elevatorsetpos(e, elevatoridlepos), // Move elevator to safe position first
            new FerrisWheelSetPos(null, ferriswheelvert) // Rotate to halfway position
            ));

    // Manually define the reverse of each entry sequence as an exit sequence
    exitSequences.put(
        4,
        new SequentialCommandGroup(
            new Elevatorsetpos(null, elevatoridlepos),
            new FerrisWheelSetPos(null, ferriswheelvert) // Reverse rotation
            // Lower elevator back to previous state
            ));

    // Define intermediate positions for transitions
    intermediatePositions.put(5, 2); // If going to position 5, first move to position 2
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

  public static boolean hasEntrySequence(int key) {
    return entrySequences.containsKey(key);
  }

  public static Command getEntrySequence(int key, Elevator elevator, FerrisWheel ferrisWheel) {
    return entrySequences.getOrDefault(key, null);
  }

  public static boolean hasExitSequence(int key) {
    return exitSequences.containsKey(key);
  }

  public static Command getExitSequence(int key, Elevator elevator, FerrisWheel ferrisWheel) {
    return exitSequences.getOrDefault(key, null);
  }
}
