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
          Map.entry("0-1", MovementType.PARALLEL),
          Map.entry("0-2", MovementType.EIFE),
          Map.entry("0-3", MovementType.PARALLEL),
          Map.entry("0-4", MovementType.PARALLEL),
          Map.entry("0-5", MovementType.EIFE),
          Map.entry("0-6", MovementType.EIFE),
          Map.entry("0-7", MovementType.EIFE),
          Map.entry("0-8", MovementType.EIFE),
          Map.entry("0-9", MovementType.PARALLEL),
          Map.entry("0-10", MovementType.PARALLEL),
          Map.entry("0-11", MovementType.EF),
          Map.entry("1-0", MovementType.FE),
          Map.entry("1-2", MovementType.FEIFE),
          Map.entry("1-3", MovementType.PARALLEL),
          Map.entry("1-4", MovementType.PARALLEL),
          Map.entry("1-5", MovementType.FEIFE),
          Map.entry("1-6", MovementType.FEIFE),
          Map.entry("1-7", MovementType.FEIFE),
          Map.entry("1-8", MovementType.FEIFE),
          Map.entry("1-9", MovementType.FE),
          Map.entry("1-10", MovementType.FE),
          Map.entry("1-11", MovementType.FE),
          Map.entry("2-0", MovementType.EIFE),
          Map.entry("2-1", MovementType.EIFEF),
          Map.entry("2-3", MovementType.E),
          Map.entry("2-4", MovementType.E),
          Map.entry("2-5", MovementType.EIFE),
          Map.entry("2-6", MovementType.EIFE),
          Map.entry("2-7", MovementType.EIFE),
          Map.entry("2-8", MovementType.EIFE),
          Map.entry("2-9", MovementType.EF),
          Map.entry("2-10", MovementType.EIFE),
          Map.entry("2-11", MovementType.EF),
          Map.entry("3-0", MovementType.EIFE),
          Map.entry("3-1", MovementType.FEF),
          Map.entry("3-2", MovementType.E),
          Map.entry("3-4", MovementType.E),
          Map.entry("3-5", MovementType.FEIFE),
          Map.entry("3-6", MovementType.PARALLEL), // either this or FEIFE
          Map.entry("3-7", MovementType.PARALLEL), // either this or FEIFE
          Map.entry("3-8", MovementType.FEIFE),
          Map.entry("3-9", MovementType.PARALLEL), // either this or FE
          Map.entry("3-10", MovementType.FEIFE),
          Map.entry("3-11", MovementType.FE),
          Map.entry("4-0", MovementType.PARALLEL),
          Map.entry("4-1", MovementType.PARALLEL),
          Map.entry("4-2", MovementType.E),
          Map.entry("4-3", MovementType.E),
          Map.entry("4-5", MovementType.FEIFE),
          Map.entry("4-6", MovementType.PARALLEL), // either this or FEIFE
          Map.entry("4-7", MovementType.PARALLEL), // either this or FEIFE
          Map.entry("4-8", MovementType.FEIFE),
          Map.entry("4-9", MovementType.FE), // either this or PARALLEL
          Map.entry("4-10", MovementType.FEIFE),
          Map.entry("4-11", MovementType.FE),
          Map.entry("5-0", MovementType.EIFE),
          Map.entry("5-1", MovementType.EIFEF),
          Map.entry("5-2", MovementType.EIFE),
          Map.entry("5-3", MovementType.EIFE),
          Map.entry("5-4", MovementType.EIFE),
          Map.entry("5-6", MovementType.EF),
          Map.entry("5-7", MovementType.EF),
          Map.entry("5-8", MovementType.F),
          Map.entry("5-9", MovementType.PARALLEL),
          Map.entry("5-10", MovementType.EIFE),
          Map.entry("5-11", MovementType.EFE),
          Map.entry("6-0", MovementType.EIFE),
          Map.entry("6-1", MovementType.PARALLEL), // double check
          Map.entry("6-2", MovementType.FEIFE),
          Map.entry("6-3", MovementType.FE),
          Map.entry("6-4", MovementType.FE),
          Map.entry("6-5", MovementType.FE),
          Map.entry("6-7", MovementType.E),
          Map.entry("6-8", MovementType.FE),
          Map.entry("6-9", MovementType.FE),
          Map.entry("6-10", MovementType.FE),
          Map.entry("6-11", MovementType.EFE),
          Map.entry("7-0", MovementType.EIFE),
          Map.entry("7-1", MovementType.PARALLEL), // double check
          Map.entry("7-2", MovementType.PARALLEL),
          Map.entry("7-3", MovementType.PARALLEL), // or FE
          Map.entry("7-4", MovementType.PARALLEL),
          Map.entry("7-5", MovementType.FE),
          Map.entry("7-6", MovementType.E),
          Map.entry("7-8", MovementType.FE),
          Map.entry("7-9", MovementType.FE),
          Map.entry("7-10", MovementType.FE),
          Map.entry("7-11", MovementType.FE),
          Map.entry("8-0", MovementType.EIFE),
          Map.entry("8-1", MovementType.EIFE),
          Map.entry("8-2", MovementType.EIFE),
          Map.entry("8-3", MovementType.EIFE),
          Map.entry("8-4", MovementType.EIFE), // maybe parallel
          Map.entry("8-5", MovementType.F),
          Map.entry("8-6", MovementType.EF),
          Map.entry("8-7", MovementType.EF),
          Map.entry("8-9", MovementType.PARALLEL),
          Map.entry("8-10", MovementType.EIFE),
          Map.entry("8-11", MovementType.EIFE),
          Map.entry("9-0", MovementType.PARALLEL),
          Map.entry("9-1", MovementType.FEIFE),
          // aint sure theres another way to pass through smoother
          Map.entry("9-2", MovementType.PARALLEL),
          Map.entry("9-3", MovementType.PARALLEL),
          Map.entry("9-4", MovementType.PARALLEL),
          Map.entry("9-5", MovementType.PARALLEL),
          Map.entry("9-6", MovementType.PARALLEL),
          Map.entry("9-7", MovementType.PARALLEL),
          Map.entry("9-8", MovementType.PARALLEL),
          Map.entry("9-10", MovementType.PARALLEL),
          Map.entry("9-11", MovementType.PARALLEL),
          // find out whats the climb position
          Map.entry("10-0", MovementType.PARALLEL),
          Map.entry("10-1", MovementType.FEIFE),
          Map.entry("10-2", MovementType.FEIFE),
          Map.entry("10-3", MovementType.FEIFE),
          Map.entry("10-4", MovementType.FEIFE),
          Map.entry("10-5", MovementType.FEIFE),
          Map.entry("10-6", MovementType.FEIFE),
          Map.entry("10-7", MovementType.FEIFE),
          Map.entry("10-8", MovementType.FEIFE),
          Map.entry("10-9", MovementType.FEIFE),
          Map.entry("10-11", MovementType.FEIFE),
          Map.entry("11-0", MovementType.PARALLEL),
          Map.entry("11-1", MovementType.FEIFE), // double check
          Map.entry("11-2", MovementType.FEIFE), // double check
          Map.entry("11-3", MovementType.PARALLEL),
          Map.entry("11-4", MovementType.PARALLEL),
          Map.entry("11-5", MovementType.FEIFE),
          Map.entry("11-6", MovementType.FE),
          Map.entry("11-7", MovementType.FE),
          Map.entry("11-8", MovementType.FEIFE),
          Map.entry("11-9", MovementType.PARALLEL),
          Map.entry("11-10", MovementType.FEIFE));

  public MoveToPositionCommand(
      Elevator elevator, FerrisWheel ferrisWheel, int currentKey, int targetKey) {

    String transitionKey = currentKey + "-" + targetKey;
    MovementType movementType = movementRules.getOrDefault(transitionKey, MovementType.FEFE);
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
          new FerrisWheelSetPos(ferrisWheel, ferrisTarget),
          new Elevatorsetpos(elevator, elevatorTarget));
    } else { // DEFAULT
      addCommands(
          new Elevatorsetpos(elevator, elevatoridlepos),
          new FerrisWheelSetPos(ferrisWheel, ferrisTarget),
          new Elevatorsetpos(elevator, elevatorTarget));
    }
  }
}
