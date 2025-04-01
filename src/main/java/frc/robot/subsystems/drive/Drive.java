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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AlgaeHexagonPositionCalculator;
import frc.robot.util.AlgaeHexagonPositionCalculator.AlgaeScoringPosition;
import frc.robot.util.HexagonPositionCalculator;
import frc.robot.util.HexagonPositionCalculator.ScoringPosition;
import frc.robot.util.LeftHexagonPositionCalculator;
import frc.robot.util.LeftHexagonPositionCalculator.LeftScoringPosition;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.RightHexagonPositionCalculator;
import frc.robot.util.RightHexagonPositionCalculator.RightScoringPosition;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public Field2d field = new Field2d();
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  double lastTx;
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Put field on dashboard
    SmartDashboard.putData("Field", field);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      // Set robot pose on the field
      field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Resets the current odometry pose based off of vision. */
  public void setVisionPose(Pose2d pose) {
    poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    System.out.println(pose.getRotation().getDegrees());
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  public Command followPath() {

    return runOnce(
        () -> {
          // Get the current robot pose from the drivetrain
          List<Waypoint> waypoints =
              PathPlannerPath.waypointsFromPoses(
                  new Pose2d(
                      getPose().getX(),
                      getPose().getY(),
                      Rotation2d.fromRadians(
                          Math.atan2(
                              findNearestPositiCommand().position.getY() - getPose().getY(),
                              findNearestPositiCommand().position.getX() - getPose().getX()))),
                  // new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromDegrees(-124)),
                  new Pose2d(
                      findNearestPositiCommand().position.getX(),
                      findNearestPositiCommand().position.getY(),
                      findNearestPositiCommand().rotation.rotateBy(Rotation2d.fromDegrees(180))));

          PathConstraints constraints =
              new PathConstraints(
                  3, 1.875, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

          // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can
          // also use
          // unlimited constraints, only limited by motor torque and nominal battery voltage

          // Create the path using the waypoints created above
          PathPlannerPath path =
              new PathPlannerPath(
                  waypoints,
                  constraints,
                  null, // The ideal starting state, this is only relevant for pre-planned paths, so
                  // can
                  // be null for on-the-fly paths.
                  new GoalEndState(
                      0.0,
                      findNearestPositiCommand()
                          .rotation) // Goal end state. You can set a holonomic rotation here. If
                  // using a
                  // differential drivetrain, the rotation will have no effect.
                  );
          // Prevent the path from being flipped if the coordinates are already correct
          path.preventFlipping = true;

          CommandScheduler.getInstance()
              .schedule(
                  new FollowPathCommand(
                      path,
                      this::getPose,
                      this::getChassisSpeeds,
                      // ChassisSpeeds, DriveFeedforwards
                      (ChassisSpeeds speeds, DriveFeedforwards feedforward) -> {
                        runVelocity(speeds);
                      },
                      new PPHolonomicDriveController(
                          new PIDConstants(5.2, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                      PP_CONFIG,
                      () -> {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                      },
                      this));
        });
  }

  public Command leftfollowPath() {

    return runOnce(
        () -> {
          // Get the current robot pose from the drivetrain
          List<Waypoint> waypoints =
              PathPlannerPath.waypointsFromPoses(
                  new Pose2d(
                      getPose().getX(),
                      getPose().getY(),
                      Rotation2d.fromRadians(
                          Math.atan2(
                              leftfindNearestPositiCommand().position.getY() - getPose().getY(),
                              leftfindNearestPositiCommand().position.getX() - getPose().getX()))),
                  // new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromDegrees(-124)),
                  new Pose2d(
                      leftfindNearestPositiCommand().position.getX(),
                      leftfindNearestPositiCommand().position.getY(),
                      leftfindNearestPositiCommand()
                          .rotation
                          .rotateBy(Rotation2d.fromDegrees(180))));

          PathConstraints constraints =
              new PathConstraints(
                  3, 1.875, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

          // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can
          // also use
          // unlimited constraints, only limited by motor torque and nominal battery voltage

          // Create the path using the waypoints created above
          PathPlannerPath path =
              new PathPlannerPath(
                  waypoints,
                  constraints,
                  null, // The ideal starting state, this is only relevant for pre-planned paths, so
                  // can
                  // be null for on-the-fly paths.
                  new GoalEndState(
                      0.0,
                      leftfindNearestPositiCommand()
                          .rotation) // Goal end state. You can set a holonomic rotation here. If
                  // using a
                  // differential drivetrain, the rotation will have no effect.
                  );
          // Prevent the path from being flipped if the coordinates are already correct
          path.preventFlipping = true;

          CommandScheduler.getInstance()
              .schedule(
                  new FollowPathCommand(
                      path,
                      this::getPose,
                      this::getChassisSpeeds,
                      // ChassisSpeeds, DriveFeedforwards
                      (ChassisSpeeds speeds, DriveFeedforwards feedforward) -> {
                        runVelocity(speeds);
                      },
                      new PPHolonomicDriveController(
                          new PIDConstants(5.2, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                      PP_CONFIG,
                      () -> {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                      },
                      this));
        });
  }

  public Command rightfollowPath() {

    return runOnce(
        () -> {
          // Get the current robot pose from the drivetrain
          List<Waypoint> waypoints =
              PathPlannerPath.waypointsFromPoses(
                  new Pose2d(
                      getPose().getX(),
                      getPose().getY(),
                      Rotation2d.fromRadians(
                          Math.atan2(
                              rightfindNearestPositiCommand().position.getY() - getPose().getY(),
                              rightfindNearestPositiCommand().position.getX() - getPose().getX()))),
                  // new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromDegrees(-124)),
                  new Pose2d(
                      rightfindNearestPositiCommand().position.getX(),
                      rightfindNearestPositiCommand().position.getY(),
                      rightfindNearestPositiCommand()
                          .rotation
                          .rotateBy(Rotation2d.fromDegrees(180))));

          PathConstraints constraints =
              new PathConstraints(
                  3, 1.875, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

          // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can
          // also use
          // unlimited constraints, only limited by motor torque and nominal battery voltage

          // Create the path using the waypoints created above
          PathPlannerPath path =
              new PathPlannerPath(
                  waypoints,
                  constraints,
                  null, // The ideal starting state, this is only relevant for pre-planned paths, so
                  // can
                  // be null for on-the-fly paths.
                  new GoalEndState(
                      0.0,
                      rightfindNearestPositiCommand()
                          .rotation) // Goal end state. You can set a holonomic rotation here. If
                  // using a
                  // differential drivetrain, the rotation will have no effect.
                  );
          // Prevent the path from being flipped if the coordinates are already correct
          path.preventFlipping = true;

          CommandScheduler.getInstance()
              .schedule(
                  new FollowPathCommand(
                      path,
                      this::getPose,
                      this::getChassisSpeeds,
                      // ChassisSpeeds, DriveFeedforwards
                      (ChassisSpeeds speeds, DriveFeedforwards feedforward) -> {
                        runVelocity(speeds);
                      },
                      new PPHolonomicDriveController(
                          new PIDConstants(5.2, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                      PP_CONFIG,
                      () -> {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                      },
                      this));
        });
  }

  public Command AlgaefollowPath() {

    return runOnce(
        () -> {
          // Get the current robot pose from the drivetrain
          List<Waypoint> waypoints =
              PathPlannerPath.waypointsFromPoses(
                  new Pose2d(
                      getPose().getX(),
                      getPose().getY(),
                      Rotation2d.fromRadians(
                          Math.atan2(
                              AlgaefindNearestPositiCommand().position.getY() - getPose().getY(),
                              AlgaefindNearestPositiCommand().position.getX() - getPose().getX()))),
                  // new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromDegrees(-124)),
                  new Pose2d(
                      AlgaefindNearestPositiCommand().position.getX(),
                      AlgaefindNearestPositiCommand().position.getY(),
                      AlgaefindNearestPositiCommand()
                          .rotation
                          .rotateBy(Rotation2d.fromDegrees(180))));

          PathConstraints constraints =
              new PathConstraints(
                  3, 1.875, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

          // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can
          // also use
          // unlimited constraints, only limited by motor torque and nominal battery voltage

          // Create the path using the waypoints created above
          PathPlannerPath path =
              new PathPlannerPath(
                  waypoints,
                  constraints,
                  null, // The ideal starting state, this is only relevant for pre-planned paths, so
                  // can
                  // be null for on-the-fly paths.
                  new GoalEndState(
                      0.0,
                      AlgaefindNearestPositiCommand()
                          .rotation) // Goal end state. You can set a holonomic rotation here. If
                  // using a
                  // differential drivetrain, the rotation will have no effect.
                  );
          // Prevent the path from being flipped if the coordinates are already correct
          path.preventFlipping = true;

          CommandScheduler.getInstance()
              .schedule(
                  new FollowPathCommand(
                      path,
                      this::getPose,
                      this::getChassisSpeeds,
                      // ChassisSpeeds, DriveFeedforwards
                      (ChassisSpeeds speeds, DriveFeedforwards feedforward) -> {
                        runVelocity(speeds);
                      },
                      new PPHolonomicDriveController(
                          new PIDConstants(5.2, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                      PP_CONFIG,
                      () -> {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                      },
                      this));
        });
  }

  public boolean isPathComplete() {
    // Check if the path is complete (based on odometry or path progress)
    return true;
  }

  public Pose2d getCurrentPose() {
    // Retrieve the current robot pose (from odometry or pose estimator)
    return new Pose2d(1.0, 1.0, new Rotation2d(0)); // Placeholder
  }

  public ScoringPosition findNearestPositiCommand() {
    double reefx = 0;
    double reefy = 0;
    Translation2d currentPosition = this.getPose().getTranslation();
    ArrayList<ScoringPosition> positions =
        HexagonPositionCalculator.calculateHexagonPositions(
            // 5, // Hexagon center X
            // 4, // Hexagon center Y
            // 1, // Radius from center to midpoint of flat side
            // 0, // X offset from midpoint
            // 0, 0

            4.489323, // blue x
            4.0259, // blue y
            13.06322, // red x
            4.0259, // red y
            0.831723 - .0381, // Radius from center to midpoint of flat side //was 0.831723
            0.5743, // X offset from midpoint
            0.3181,
            .0079

            // Y offset from midpoint
            ,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);

    // 3.103, 3.706,
    ScoringPosition nearest =
        HexagonPositionCalculator.findNearestPosition(currentPosition, positions, reefx, reefy);

    return nearest;
    // System.out.println("Nearest Scoring Position: " + nearest);
    // Implement logic to drive the robot to the nearest position
    // x 4.489323
    // y 4.0259

  }

  public LeftScoringPosition leftfindNearestPositiCommand() {
    double reefx = 0;
    double reefy = 0;
    Translation2d currentPosition = this.getPose().getTranslation();
    ArrayList<LeftScoringPosition> positions =
        LeftHexagonPositionCalculator.calculateHexagonPositions(
            // 5, // Hexagon center X
            // 4, // Hexagon center Y
            // 1, // Radius from center to midpoint of flat side
            // 0, // X offset from midpoint
            // 0, 0

            4.489323, // blue x
            4.0259, // blue y
            13.06322, // red x
            4.0259, // red y
            0.831723 - .0381, // Radius from center to midpoint of flat side //was 0.831723
            0.5743, // X offset from midpoint
            0.3181,
            .0079

            // Y offset from midpoint
            ,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);

    // 3.103, 3.706,
    LeftScoringPosition nearest =
        LeftHexagonPositionCalculator.findNearestPosition(currentPosition, positions, reefx, reefy);

    return nearest;
    // System.out.println("Nearest Scoring Position: " + nearest);
    // Implement logic to drive the robot to the nearest position
    // x 4.489323
    // y 4.0259

  }

  public RightScoringPosition rightfindNearestPositiCommand() {
    double reefx = 0;
    double reefy = 0;
    Translation2d currentPosition = this.getPose().getTranslation();
    ArrayList<RightScoringPosition> positions =
        RightHexagonPositionCalculator.calculateHexagonPositions(
            // 5, // Hexagon center X
            // 4, // Hexagon center Y
            // 1, // Radius from center to midpoint of flat side
            // 0, // X offset from midpoint
            // 0, 0

            4.489323, // blue x
            4.0259, // blue y
            13.06322, // red x
            4.0259, // red y
            0.831723 - .0381, // Radius from center to midpoint of flat side //was 0.831723
            0.5743, // X offset from midpoint
            0.3181,
            .0079

            // Y offset from midpoint
            ,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);

    // 3.103, 3.706,
    RightScoringPosition nearest =
        RightHexagonPositionCalculator.findNearestPosition(
            currentPosition, positions, reefx, reefy);

    return nearest;
    // System.out.println("Nearest Scoring Position: " + nearest);
    // Implement logic to drive the robot to the nearest position
    // x 4.489323
    // y 4.0259

  }

  public AlgaeScoringPosition AlgaefindNearestPositiCommand() {
    double reefx = 0;
    double reefy = 0;
    Translation2d currentPosition = this.getPose().getTranslation();
    ArrayList<AlgaeScoringPosition> positions =
        AlgaeHexagonPositionCalculator.calculateHexagonPositions(
            // 5, // Hexagon center X
            // 4, // Hexagon center Y
            // 1, // Radius from center to midpoint of flat side
            // 0, // X offset from midpoint
            // 0, 0

            4.489323, // blue x
            4.0259, // blue y
            13.06322, // red x
            4.0259, // red y
            0.831723 - .0381, // Radius from center to midpoint of flat side //was 0.831723
            0.56, // X offset from midpoint
            0.3181,
            4.0259 - 4.18

            // pick x4.33m

            // Y offset from midpoint
            ,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);

    // 3.103, 3.706,
    AlgaeScoringPosition nearest =
        AlgaeHexagonPositionCalculator.findNearestPosition(
            currentPosition, positions, reefx, reefy);

    return nearest;
    // System.out.println("Nearest Scoring Position: " + nearest);
    // Implement logic to drive the robot to the nearest position
    // x 4.489323
    // y 4.0259

  }

  public void cancelPath() {
    if (currentPathCommand != null && currentPathCommand.isScheduled()) {
      currentPathCommand.cancel();
    }
  }

  private Command currentPathCommand = null;
}
