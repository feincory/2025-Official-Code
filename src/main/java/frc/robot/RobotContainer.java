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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ClearElevator;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HomeLiftCommand;
import frc.robot.commands.MoveToPositionCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CLIMBER;
import frc.robot.subsystems.CoralGround;
import frc.robot.subsystems.CoralIntake;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Elevator m_Elevator = new Elevator();
  public final CLIMBER m_climber = new CLIMBER();
  public final CoralIntake m_CoralIntake = new CoralIntake();
  public final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
  public final FerrisWheel m_FerrisWheel = new FerrisWheel();
  public final CoralGround m_coralground = new CoralGround();
  private final Vision vision;
  // Controller
  // private final CommandXboxController testcontroller = new CommandXboxController(2);
  private final CommandXboxController controller = new CommandXboxController(1);
  public final CommandJoystick m_drivercontroller = new CommandJoystick(0);
  private int currentKey = 0; // Track the last known positio

  private final SendableChooser<Command> AutonChoice;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Coral Retreive", new InstantCommand(() -> moveToPosition(1)));
    NamedCommands.registerCommand("Coral Outtake", new InstantCommand(m_FerrisWheel::coralout));
    NamedCommands.registerCommand("Coral L4", new InstantCommand(() -> moveToPosition(4)));
    NamedCommands.registerCommand("Coral Intake", new InstantCommand(m_FerrisWheel::coralin));
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

    AutonChoice = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", AutonChoice);

    // Configure the button bindings
    configureButtonBindings();

    createautoDashboards();
  }

  public void createautoDashboards() {
    ShuffleboardTab autotab = Shuffleboard.getTab("Auto");
    autotab.add("Auto Chooser", AutonChoice).withSize(1, 1).withPosition(4, 0);
  }

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
    // testcontroller
    //     .a()
    //     .onTrue(
    //         DriveCommands.AutoLineUp(
    //             drive,
    //             () -> (((vision.getTargetY(0).getDegrees() - 4)) * .5),
    //             () -> ((vision.getTargetX(0).getDegrees() + 2.05) * -2.5),
    //             () -> -m_drivercontroller.getRawAxis(3)))
    //     .onFalse(Commands.runOnce(drive::stop, drive));
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

    m_drivercontroller
        .button(11)
        .onFalse(new InstantCommand(m_coralground::shootpos)); // run to ground

    m_drivercontroller.button(10).onFalse(new InstantCommand(m_coralground::shootpos)); // shoot

    m_drivercontroller
        .button(11)
        .onTrue(new InstantCommand(m_coralground::pickupos)); // place coral on reef

    m_drivercontroller.button(10).onTrue(new InstantCommand(m_coralground::storagepos));

    m_drivercontroller
        .button(13)
        .onTrue(new InstantCommand(m_coralground::runspinner))
        .onFalse(new InstantCommand(m_coralground::stopspinner));

    m_drivercontroller
        .button(13)
        .onTrue(new InstantCommand(m_FerrisWheel::coralout))
        .onFalse(new InstantCommand(m_FerrisWheel::coralstop));

    m_drivercontroller // coral ground pickup homing
        .button(15)
        .whileTrue(new RunCommand(m_coralground::homingroutine))
        .onFalse(new InstantCommand(m_coralground::stop));

    // coral intake button binding
    controller
        .rightBumper()
        .onTrue(new InstantCommand(m_FerrisWheel::coralout))
        .onFalse(new InstantCommand(m_FerrisWheel::coralstop));
    controller
        .leftBumper()
        .onTrue(new InstantCommand(m_FerrisWheel::coralin))
        .onFalse(new InstantCommand(m_FerrisWheel::coralstop));

    // algae intake button binding
    controller
        .rightTrigger()
        .onTrue(new InstantCommand(m_FerrisWheel::algaeout))
        .onFalse(new InstantCommand(m_FerrisWheel::algaestop));
    controller
        .leftTrigger()
        .onTrue(new InstantCommand(m_FerrisWheel::algaein))
        .onFalse(new InstantCommand(m_FerrisWheel::algaehold));
    // climber
    controller
        .leftStick()
        .onTrue(new InstantCommand(m_climber::climbup))
        .onFalse(new InstantCommand(m_climber::climbstop));

    controller
        .rightStick()
        .onTrue(new InstantCommand(m_climber::climbdown))
        .onFalse(new InstantCommand(m_climber::climbstop));

    controller.leftStick().onTrue(new InstantCommand(m_coralground::stopspinner));

    controller.rightStick().onTrue(new InstantCommand(m_coralground::stopspinner));

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
    // testcontroller.start().onTrue(new InstantCommand(() -> moveToPosition(11)));
  }

  private void moveToPosition(int targetKey) {
    new MoveToPositionCommand(m_Elevator, m_FerrisWheel, currentKey, targetKey).schedule();
    // Update currentKey right after scheduling the command
    currentKey = targetKey;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    new InstantCommand(m_coralground::initstoragepos);
    return AutonChoice.getSelected();
    // return autoChooser.get();
  }
}
