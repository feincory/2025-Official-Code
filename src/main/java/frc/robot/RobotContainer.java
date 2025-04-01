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

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ClearElevator;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HomeLiftCommand;
import frc.robot.commands.MoveToPositionCommand;
import frc.robot.commands.WaitForGamePieceCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CLIMBER;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FerrisWheel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(1);
  public final CommandJoystick m_drivercontroller = new CommandJoystick(0);
  // Subsystems
  private final Drive drive;
  public final Elevator m_Elevator = new Elevator();
  public final CLIMBER m_climber = new CLIMBER();
  public final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
  public final FerrisWheel m_FerrisWheel = new FerrisWheel();
  public final CANdleSystem m_CANdleSystem = new CANdleSystem();
  // public final CoralGround m_coralground = new CoralGround();
  private final Vision vision;

  // Controller
  // private final CommandXboxController testcontroller = new CommandXboxController(2);

  private int currentKey = 0; // Track the last known positio
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  // private final SendableChooser<Command> AutonChoice;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Coral Retreive", new InstantCommand(() -> moveToPosition(1)));
    NamedCommands.registerCommand("Coral Outtake", new InstantCommand(m_FerrisWheel::coralout));
    NamedCommands.registerCommand("Coral L4", new InstantCommand(() -> moveToPosition(4)));
    NamedCommands.registerCommand("Coral Intake", new InstantCommand(m_FerrisWheel::coralin));
    NamedCommands.registerCommand("Settle For Place", new WaitCommand(.5));
    NamedCommands.registerCommand("Settle For Retreive", new WaitCommand(.3));
    NamedCommands.registerCommand(
        "Wait for Game Piece", new WaitForGamePieceCommand(m_FerrisWheel.getSensor(), 2.5));
    NamedCommands.registerCommand(
        "Check L4 Clear", new WaitForGamePieceCommand(m_FerrisWheel.getL4Sensor(), 15));

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision = null;
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = null;
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    createautoDashboards();
  }

  public void createautoDashboards() {
    ShuffleboardTab Prematch = Shuffleboard.getTab("Pre-Match");
    // Prematch.add(
    //         "Reset Pose",
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setVisionPose(
    //                         new Pose2d(
    //                             LimelightHelpers.getBotPose2d_wpiBlue("limelight-front")
    //                                 .getTranslation(),
    //                             new Rotation2d(
    //                                 LimelightHelpers.getBotPose2d_wpiBlue("limelight-front")
    //                                     .getRotation()
    //                                     .getDegrees()))),
    //                 drive)
    //             .ignoringDisable(true))
    //     .withWidget(BuiltInWidgets.kCommand);

    Prematch.add(
            "Test Game Piece Wait",
            new WaitForGamePieceCommand(m_FerrisWheel.getSensor(), 5)
                .ignoringDisable(true)) // Ensure you pass the sensor instance
        .withWidget(BuiltInWidgets.kCommand);

    // Prematch.add("Coral Prox Detect", new CoralInProx(m_FerrisWheel, 1).ignoringDisable(true))
    //     .withWidget(BuiltInWidgets.kCommand);
    // ShuffleboardTab Prematch = Shuffleboard.getTab("Pre-Match");
    // Prematch.add(
    //         "Reset Pose",
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setVisionPose(
    //                         LimelightHelpers.getBotPose2d_wpiBlue("limelight-front")),
    //                 drive)
    //             .ignoringDisable(true))
    //     .withWidget(BuiltInWidgets.kCommand);
    // ShuffleboardTab autotab = Shuffleboard.getTab("Auto");
    // autotab.add("Auto Chooser", AutonChoice).withSize(1, 1).withPosition(4, 0);
  }

  // new
  // InstantCommand(drive.setVisionPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight-front"))))
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> m_drivercontroller.getRawAxis(1),
            () -> -m_drivercontroller.getRawAxis(0),
            () -> -m_drivercontroller.getRawAxis(3)));

    // Switch to X pattern when X button is pressed
    m_drivercontroller.button(16).onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Bind the command to run while the button is held down:
    // m_drivercontroller
    //     .button(12)
    //     .onTrue(
    //         DriveCommands.AutoLineUp(
    //             drive,
    //             () -> (((vision.getTargetY(0).getDegrees() - 0)) * 0), // ty is acutally ta
    //             () -> ((vision.getTargetX(0).getDegrees() + 2.05) * 0), // tx is tx
    //             () -> -m_drivercontroller.getRawAxis(3)))
    //     .onFalse(Commands.runOnce(drive::stop, drive));

    // // Bind the command to run while the button is held down:
    m_drivercontroller
        .button(19)
        .onTrue(DriveCommands.reeflineup(drive, () -> 0, () -> -.5, () -> 0))
        .onFalse(Commands.runOnce(drive::stop, drive));

    m_drivercontroller
        .button(20)
        .onTrue(DriveCommands.reeflineup(drive, () -> 0, () -> .5, () -> 0))
        .onFalse(Commands.runOnce(drive::stop, drive));

    m_drivercontroller
        .button(22)
        .onTrue(DriveCommands.reeflineup(drive, () -> -.5, () -> 0, () -> 0))
        .onFalse(Commands.runOnce(drive::stop, drive));

    m_drivercontroller
        .button(21)
        .onTrue(DriveCommands.reeflineup(drive, () -> .5, () -> 0, () -> 0))
        .onFalse(Commands.runOnce(drive::stop, drive));

    // Reset gyro to 0° when B button is pressed
    m_drivercontroller
        .button(14)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // m_drivercontroller
    //     .button(13)
    //     .onTrue(new InstantCommand(m_FerrisWheel::dipsettrue))
    //     .onFalse(new InstantCommand(m_FerrisWheel::dipsetfalse));

    m_drivercontroller
        .button(12)
        .onTrue(new InstantCommand(m_FerrisWheel::coraloutslow))
        .onFalse(new InstantCommand(m_FerrisWheel::coralhold));

    // m_drivercontroller // coral ground pickup homing
    //     .button(15)
    //     .whileTrue(new RunCommand(m_coralground::homingroutine))
    //     .onFalse(new InstantCommand(m_coralground::stop));

    // coral intake button binding
    controller
        .rightBumper()
        .onTrue(new InstantCommand(m_FerrisWheel::coraloutslow))
        .onFalse(new InstantCommand(m_FerrisWheel::coralstop));
    controller
        .leftBumper()
        .onTrue(new InstantCommand(m_FerrisWheel::coralin))
        .onTrue(
            new InstantCommand(
                () -> m_CANdleSystem.changeAnimation(CANdleSystem.AnimationTypes.Twinkle)))
        .onFalse(new InstantCommand(m_FerrisWheel::coralhold));

    // algae intake button binding
    controller
        .rightTrigger()
        .onTrue(new InstantCommand(m_FerrisWheel::algaeout))
        .onFalse(new InstantCommand(m_FerrisWheel::algaestop));
    controller
        .leftTrigger()
        .onTrue(new InstantCommand(m_FerrisWheel::algaein))
        .onTrue(
            new InstantCommand(
                () -> m_CANdleSystem.changeAnimation(CANdleSystem.AnimationTypes.Fire)))
        .onFalse(new InstantCommand(m_FerrisWheel::algaehold));
    // // climber
    controller
        .leftStick()
        .onTrue(new InstantCommand(m_climber::climbup))
        .onFalse(new InstantCommand(m_climber::climbhold));

    controller
        .rightStick()
        .onTrue(new InstantCommand(m_climber::climbdown))
        .onFalse(new InstantCommand(m_climber::climbstop));

    controller
        .rightStick()
        .onTrue(new InstantCommand(m_climber::funnelrelease))
        .onTrue(
            new InstantCommand(
                () -> m_CANdleSystem.changeAnimation(CANdleSystem.AnimationTypes.Rainbow)));

    controller.leftStick().onTrue(new InstantCommand(m_climber::funnelrelease));

    // ferris wheel controls

    m_drivercontroller // elevator homing
        .button(16)
        .onTrue(
            new SequentialCommandGroup(
                new ClearElevator(m_Elevator),
                new InstantCommand(m_FerrisWheel::startingposition),
                new HomeLiftCommand(m_Elevator, 0),
                new InstantCommand(m_FerrisWheel::ferrisstop)));

    controller.start().onTrue(new InstantCommand(() -> moveToPosition(0)));
    controller.back().onTrue(new InstantCommand(() -> moveToPosition(8)));
    controller.povLeft().onTrue(new InstantCommand(() -> moveToPosition(6)));
    controller.povRight().onTrue(new InstantCommand(() -> moveToPosition(7)));
    controller.povUp().onTrue(new InstantCommand(() -> moveToPosition(9)));
    controller.povDown().onTrue(new InstantCommand(() -> moveToPosition(5)));
    controller.a().onTrue(new InstantCommand(() -> moveToPosition(1)));
    controller.x().onTrue(new InstantCommand(() -> moveToPosition(2)));
    controller.b().onTrue(new InstantCommand(() -> moveToPosition(3)));
    controller.y().onTrue(new InstantCommand(() -> moveToPosition(4)));
    m_drivercontroller.button(4).onTrue(new InstantCommand(() -> moveToPosition(12)));
    m_drivercontroller.button(4).onTrue(new InstantCommand(m_climber::okaytorelease));

    m_drivercontroller.button(2).onTrue(drive.leftfollowPath());
    m_drivercontroller.button(2).onFalse(new InstantCommand(drive::stop));
    // m_drivercontroller.button(1).onFalse(autoChooser.followPath(null).cancel());

    m_drivercontroller.button(3).onTrue(drive.rightfollowPath());
    m_drivercontroller.button(3).onFalse(new InstantCommand(drive::stop));

    m_drivercontroller.button(13).onTrue(drive.AlgaefollowPath());
    m_drivercontroller.button(13).onFalse(new InstantCommand(drive::stop));

    // m_drivercontroller.button(7).onTrue(new InstantCommand(() ->
    // drive.findNearestPositiCommand()));
    // m_drivercontroller.button(7).onFalse(new InstantCommand(drive::stop));
  }

  private void moveToPosition(int targetKey) {
    new MoveToPositionCommand(m_Elevator, m_FerrisWheel, currentKey, targetKey).schedule();
    // Update currentKey right after scheduling the command
    currentKey = targetKey;
  }

  public Command setcoralgrouninit() {
    // return new InstantCommand(m_coralground::initstoragepos);
    return null;
    // Update currentKey right after scheduling the command
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // new InstantCommand(m_coralground::initstoragepos);
    // return AutonChoice.getSelected();
    return autoChooser.get();
  }

  public void createpathforteleop() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic
    // rotation.
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(
                drive.getPose().getTranslation().getX(),
                drive.getPose().getTranslation().getY(),
                drive.getPose().getRotation()),
            new Pose2d(3.103, 3.706, Rotation2d.fromDegrees(180)));

    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
    // unlimited constraints, only limited by motor torque and nominal battery voltage

    // Create the path using the waypoints created above
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                Rotation2d.fromDegrees(
                    180)) // Goal end state. You can set a holonomic rotation here. If using a
            // differential drivetrain, the rotation will have no effect.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
  }
}
