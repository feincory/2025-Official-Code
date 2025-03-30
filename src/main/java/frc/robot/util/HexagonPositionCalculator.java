package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;

public class HexagonPositionCalculator {

  public static class ScoringPosition {
    public Translation2d position;
    public Rotation2d rotation;

    public ScoringPosition(Translation2d position, Rotation2d rotation) {
      this.position = position;
      this.rotation = rotation;
    }

    @Override
    public String toString() {
      return "Position: ("
          + position.getX()
          + ", "
          + position.getY()
          + "), Rotation: "
          + rotation.getDegrees()
          + "°";
    }
  }

  public static ArrayList<ScoringPosition> calculateHexagonPositions(
      double centerblueX,
      double centerblueY,
      double centerredX,
      double centerredY,
      double radius,
      double xOffset,
      double yOffset1,
      double yOffset2,
      boolean redside) {
    ArrayList<ScoringPosition> positions = new ArrayList<>();
    double angleIncrement = Math.toRadians(60);

    for (int side = 0; side < 7; side++) {
      // double angle = 0;
      double angle = (side * angleIncrement);
      double angleradians1 = angle + Math.atan((+yOffset1) / (radius + xOffset));
      double hypot1 = Math.hypot(radius + xOffset, yOffset1);
      double angleradians2 = angle + Math.atan((-yOffset2) / (radius + xOffset));
      double hypot2 = Math.hypot(radius + xOffset, yOffset2);

      double targetx1 = hypot1 * Math.cos(angleradians1);
      double targety1 = hypot1 * Math.sin(angleradians1);
      double targetx2 = hypot2 * Math.cos(angleradians2);
      double targety2 = hypot2 * Math.sin(angleradians2);
      double rotationAngle = angle;
      Rotation2d rotation = new Rotation2d(rotationAngle);

      double netbluex = 7.58;
      double netbluey1 = 7.078;
      double netbluey2 = 5.213;
      double netredx = 8.77 + (8.77 - netbluex);
      double netredy1 = 4.2059 - (netbluey1 - 4.0259);
      double netredy2 = 4.2059 - (netbluey2 - 4.2059);

      if (side == 6) {
        if (redside) {
          // 8.77 x
          // 4.0259 y
          positions.add(
              new ScoringPosition(new Translation2d(netredx, netredy1), Rotation2d.fromDegrees(0)));
          positions.add(
              new ScoringPosition(new Translation2d(netredx, netredy2), Rotation2d.fromDegrees(0)));
        } else {
          positions.add(
              new ScoringPosition(
                  new Translation2d(netbluex, netbluey1), Rotation2d.fromDegrees(180)));
          positions.add(
              new ScoringPosition(
                  new Translation2d(netbluex, netbluey2), Rotation2d.fromDegrees(180)));
        }
        // positions.add(new ScoringPosition(new Translation2d(netbluex, netbluey1),
        // Rotation2d.fromDegrees(180)));
        // positions.add(new ScoringPosition(new Translation2d(netbluex, netbluey2),
        // Rotation2d.fromDegrees(180)));
      } else {
        if (redside) {
          double pos1X = targetx1 + centerredX;
          double pos1Y = targety1 + centerredY;
          double pos2X = targetx2 + centerredX;
          double pos2Y = targety2 + centerredY;
          positions.add(new ScoringPosition(new Translation2d(pos1X, pos1Y), rotation));
          positions.add(new ScoringPosition(new Translation2d(pos2X, pos2Y), rotation));
        } else {
          double pos1X = targetx1 + centerblueX;
          double pos1Y = targety1 + centerblueY;
          double pos2X = targetx2 + centerblueX;
          double pos2Y = targety2 + centerblueY;
          positions.add(new ScoringPosition(new Translation2d(pos1X, pos1Y), rotation));
          positions.add(new ScoringPosition(new Translation2d(pos2X, pos2Y), rotation));
        }
      }
      // System.out.print("pos1x" + pos1X);
      // System.out.print("pos1y" + pos1Y);

      // System.out.print("pos2x" + pos2X);
      // System.out.print("pos2y" + pos2Y);

    }
    System.out.print("Positions" + positions);
    return positions;
  }

  public static ScoringPosition findNearestPosition(
      Translation2d currentPosition,
      ArrayList<ScoringPosition> positions,
      double reefx,
      double reefy) {
    double minDistance = Double.MAX_VALUE;
    ScoringPosition nearestPosition = null;

    for (ScoringPosition position : positions) {
      double distance = currentPosition.getDistance(position.position);
      if (distance < minDistance) {
        minDistance = distance;
        nearestPosition = position;
      }
    }

    return nearestPosition;
  }
}

// center of blue reef
// x 4.489323
// y 4.0259
