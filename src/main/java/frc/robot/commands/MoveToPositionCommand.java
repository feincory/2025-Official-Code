package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FerrisWheel;
import frc.robot.util.MovementType;
import java.util.Map;

public class MoveToPositionCommand extends SequentialCommandGroup {

  // Define target positions for elevator and ferris wheel
  private static final Map<Integer, Double> elevatorPositions =
      Map.ofEntries(
          Map.entry(0, elevatorstow),
          Map.entry(1, elevatorRetreivepos),
          Map.entry(2, elevatorL2pos),
          Map.entry(3, elevatorL3pos),
          Map.entry(4, elevatorL4pos),
          Map.entry(5, elevatorRetreiveAlgaepos),
          Map.entry(6, elevatorL2Algaepos),
          Map.entry(7, elevatorL3Algaepos),
          Map.entry(8, elevatorprocessor),
          Map.entry(9, elevatornetpos),
          Map.entry(10, elevatorclimb),
          Map.entry(11, elevatoridlepos));

  private static final Map<Integer, Double> ferrisWheelPositions =
      Map.ofEntries(
          Map.entry(0, ferriswheelvert),
          Map.entry(1, ferriscoralretreive),
          Map.entry(2, ferriscoralplace),
          Map.entry(3, ferriscoralplace),
          Map.entry(4, ferriscoralplace),
          Map.entry(5, ferrisalageretreive),
          Map.entry(6, ferrisAlgaeReefPic),
          Map.entry(7, ferrisAlgaeReefPic),
          Map.entry(8, ferrisalgaeprocessor),
          Map.entry(9, ferrisAlgaeNet),
          Map.entry(10, ferriswheelvert),
          Map.entry(11, ferriswheelvert));
  //   // Define movement rules directly in this command
  //   private static final Map<String, MovementType> movementRules =
  //       Map.of(
  //           "0-1", MovementType.EF,
  //           "1-4", MovementType.PARALLEL,
  //           "4-1", MovementType.FEF,
  //           "3-1", MovementType.PARALLEL,
  //           "1-3", MovementType.FEF,
  //           "2-5", MovementType.FEFE,
  //           "3-6", MovementType.PARALLEL,
  //           "4-7", MovementType.EFE,
  //           "5-8", MovementType.EFEF);

  // Define movement rules directly in this command
  private static final Map<String, MovementType> movementRules =
      Map.ofEntries(
          Map.entry("0-1", MovementType.EF),
          Map.entry("1-3", MovementType.PARALLEL),
          Map.entry("1-2", MovementType.FEIFE),
          Map.entry("1-4", MovementType.PARALLEL),
          Map.entry("1-5", MovementType.FEIFE),
          Map.entry("1-6", MovementType.FEIFE),
          Map.entry("1-7", MovementType.FEIFE),
          Map.entry("1-8", MovementType.FEIFE),
          Map.entry("1-9", MovementType.FE),
          Map.entry("1-10", MovementType.FE),
          Map.entry("1-11", MovementType.FE),
          Map.entry("2-1", MovementType.FEIFE),
          Map.entry("2-5", MovementType.FEIFE),
          Map.entry("2-6", MovementType.FEIFE),
          Map.entry("2-7", MovementType.FEIFE),
          Map.entry("2-8", MovementType.FEIFE),
          Map.entry("2-9", MovementType.FE),
          Map.entry("2-10", MovementType.FE),
          Map.entry("2-11", MovementType.FE),
          Map.entry("4-1", MovementType.PARALLEL),
          Map.entry("3-1", MovementType.FEF));

  public MoveToPositionCommand(
      Elevator elevator, FerrisWheel ferrisWheel, int currentKey, int targetKey) {

    String transitionKey = currentKey + "-" + targetKey;
    MovementType movementType = movementRules.getOrDefault(transitionKey, MovementType.EF);
    double elevatorTarget = elevatorPositions.getOrDefault(targetKey, elevatorstow);
    double ferrisTarget = ferrisWheelPositions.getOrDefault(targetKey, ferriswheelvert);

    // Debugging Output
    System.out.println("Executing move from " + currentKey + " to " + targetKey);
    System.out.println("Elevator Target Position: " + elevatorTarget);
    System.out.println("Ferris Wheel Target Position: " + ferrisTarget);
    System.out.println("Movement Type: " + movementType);

    if (movementType == MovementType.E) {
      addCommands(new Elevatorsetpos(elevator, elevatorTarget));
    } else if (movementType == MovementType.F) {
      addCommands(new FerrisWheelSetPos(ferrisWheel, ferrisTarget));
    } else if (movementType == MovementType.FE) {
      addCommands(
          new FerrisWheelSetPos(ferrisWheel, ferrisTarget),
          new Elevatorsetpos(elevator, elevatorTarget));
    } else if (movementType == MovementType.EF) {
      addCommands(
          new Elevatorsetpos(elevator, elevatorTarget),
          new FerrisWheelSetPos(ferrisWheel, ferrisTarget));
    } else if (movementType == MovementType.FEF) {
      addCommands(
          new FerrisWheelSetPos(ferrisWheel, ferriswheelvert),
          new Elevatorsetpos(elevator, elevatorTarget),
          new FerrisWheelSetPos(ferrisWheel, ferrisTarget));
    } else if (movementType == MovementType.EFE) {
      addCommands(
          new Elevatorsetpos(elevator, elevatorTarget),
          new FerrisWheelSetPos(ferrisWheel, ferrisTarget),
          new Elevatorsetpos(elevator, elevatorTarget));
    } else if (movementType == MovementType.FEFE) {
      addCommands(
          new FerrisWheelSetPos(ferrisWheel, ferriswheelvert),
          new Elevatorsetpos(elevator, elevatorTarget),
          new FerrisWheelSetPos(ferrisWheel, ferrisTarget),
          new Elevatorsetpos(elevator, elevatorTarget));
    } else if (movementType == MovementType.EFEF) {
      addCommands(
          new Elevatorsetpos(elevator, elevatorTarget),
          new FerrisWheelSetPos(ferrisWheel, ferriswheelvert),
          new Elevatorsetpos(elevator, elevatorTarget),
          new FerrisWheelSetPos(ferrisWheel, ferrisTarget));
    } else if (movementType == MovementType.EIFEF) {
      addCommands(
          new Elevatorsetpos(elevator, elevatoridlepos),
          new FerrisWheelSetPos(ferrisWheel, ferriswheelvert),
          new Elevatorsetpos(elevator, elevatorTarget),
          new FerrisWheelSetPos(ferrisWheel, ferrisTarget));

    } else if (movementType == MovementType.FEIFE) {
      addCommands(
          new FerrisWheelSetPos(ferrisWheel, ferriswheelvert),
          new Elevatorsetpos(elevator, elevatoridlepos),
          new FerrisWheelSetPos(ferrisWheel, ferrisTarget),
          new Elevatorsetpos(elevator, elevatorTarget));
    } else if (movementType == MovementType.EIFE) {
      addCommands(
          new Elevatorsetpos(elevator, elevatoridlepos),
          new FerrisWheelSetPos(ferrisWheel, ferriswheelvert),
          new Elevatorsetpos(elevator, elevatorTarget));
    } else { // PARALLEL
      addCommands(
          new ParallelCommandGroup(
              new Elevatorsetpos(elevator, elevatorTarget),
              new FerrisWheelSetPos(ferrisWheel, ferrisTarget)));
    }
  }
}
