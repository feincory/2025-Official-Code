// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // ferris wheel PIDs
  public static double kfPc = 40; // was 40
  public static double kfIc = 0; //
  public static double kfDc = 2;
  public static double kfGc = 0; // Gravity

  // Algae Elevator constances
  public static double elevatorRetreiveAlgaepos = -2.10;
  public static double elevatoridlepos = -15.5;
  public static double elevatoridleposL2 = -15.5;
  public static double elevatorL2Algaepos = -9.73;
  public static double elevatorL3Algaepos = -17;
  public static double elevatornetpos = -27;
  public static double ferrisalageretreive = .918;
  public static double ferrisAlgaeReefPic = .814;
  public static double ferrisAlgaeNet = .615;
  public static double ferriswheelvert = .524;
  public static double ferrisalgaeprocessor = .86;
  public static double elevatorprocessor = -3;

  // Coral Elevator Positions
  public static double elevatorRetreivepos = -4.9; // WAS 4.9

  public static double elevatorL2pos = -6.9;
  public static double elevatorL3pos = -15;
  public static double elevatorL4pos = -27.6; // was -27.3
  public static double ferriscoralplace = .172; // was .179
  public static double ferriscoralplaceL2 = .175;
  public static double ferriscoralretreive = .620;

  // climb
  public static double elevatorstow = -1;
  public static double elevatorclimb = -1;

  // Double set position for Algae L2 Movement
  // public static double elevatorL2combopos = -6.9;

}
