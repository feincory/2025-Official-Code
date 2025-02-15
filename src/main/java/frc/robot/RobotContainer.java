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

import static frc.robot.Constants.elevatorL2Algaepos;
import static frc.robot.Constants.elevatorL3Algaepos;
import static frc.robot.Constants.elevatorL3pos;
import static frc.robot.Constants.elevatorL4pos;
import static frc.robot.Constants.elevatorRetreivepos;
import static frc.robot.Constants.elevatoridlepos;
import static frc.robot.Constants.elevatornetpos;
import static frc.robot.Constants.ferrisAlgaeNet;
import static frc.robot.Constants.ferrisAlgaeReefPic;
import static frc.robot.Constants.ferriscoralplace;
import static frc.robot.Constants.ferriscoralretreive;
import static frc.robot.Constants.ferriswheelvert;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ClearElevator;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Elevatorsetpos;
import frc.robot.commands.FerrisWheelSetPos;
import frc.robot.commands.FerrisWheelVertical;
import frc.robot.commands.HomeLiftCommand;
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
  // Controller
  private final CommandXboxController testcontroller = new CommandXboxController(2);
  private final CommandXboxController controller = new CommandXboxController(1);
  public final CommandJoystick m_drivercontroller = new CommandJoystick(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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

    // drivetrain.setDefaultCommand(
    //     // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() ->
    //         drive.withVelocityX(-testcontroller.getLeftY() * MaxSpeed) // Drive forward with
    // negative Y (forward)
    //             .withVelocityY(-testcontroller.getLeftX() * MaxSpeed) // Drive left with negative
    // X (left)
    //             .withRotationalRate(-testcontroller.getRightX() * MaxAngularRate) // Drive
    // counterclockwise with negative X (left)
    //     )

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    m_drivercontroller.button(16).onTrue(Commands.runOnce(drive::stopWithX, drive));

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

    controller
        .start()
        .onTrue(
            new SequentialCommandGroup(
                new ClearElevator(m_Elevator),
                new InstantCommand(m_FerrisWheel::startingposition),
                new HomeLiftCommand(m_Elevator, 0),
                new InstantCommand(m_FerrisWheel::ferrisstop)));

    // coral intake button binding
    controller
        .rightBumper()
        .onTrue(new InstantCommand(m_CoralIntake::coralin))
        .onFalse(new InstantCommand(m_CoralIntake::coralstop));
    controller
        .leftBumper()
        .onTrue(new InstantCommand(m_CoralIntake::coralout))
        .onFalse(new InstantCommand(m_CoralIntake::coralstop));

    // algae intake button binding
    controller
        .rightTrigger()
        .onTrue(new InstantCommand(m_AlgaeIntake::algaein))
        .onFalse(new InstantCommand(m_AlgaeIntake::algaestop));
    controller
        .leftTrigger()
        .onTrue(new InstantCommand(m_AlgaeIntake::algaeout))
        .onFalse(new InstantCommand(m_AlgaeIntake::algaestop));

    // ferris wheel controls
    controller
        .y()
        .onTrue(
            new SequentialCommandGroup(
                new FerrisWheelVertical(m_FerrisWheel),
                new Elevatorsetpos(m_Elevator, elevatoridlepos),
                new ParallelCommandGroup(
                    new Elevatorsetpos(m_Elevator, elevatorL4pos),
                    new FerrisWheelSetPos(m_FerrisWheel, ferriscoralplace))));
    controller
        .b()
        .onTrue(
            new SequentialCommandGroup(
                new FerrisWheelVertical(m_FerrisWheel),
                new Elevatorsetpos(m_Elevator, elevatoridlepos),
                new ParallelCommandGroup(
                    new Elevatorsetpos(m_Elevator, elevatorL3pos),
                    new FerrisWheelSetPos(m_FerrisWheel, ferriscoralplace))));

    // algae net placement
    controller
        .povUp()
        .onTrue(
            new SequentialCommandGroup(
                new FerrisWheelVertical(m_FerrisWheel),
                new Elevatorsetpos(m_Elevator, elevatoridlepos),
                new ParallelCommandGroup(
                    new Elevatorsetpos(m_Elevator, elevatornetpos),
                    new FerrisWheelSetPos(m_FerrisWheel, ferrisAlgaeNet))));

    controller
        .povLeft()
        .onTrue(
            new SequentialCommandGroup(
                new FerrisWheelVertical(m_FerrisWheel),
                new Elevatorsetpos(m_Elevator, elevatoridlepos),
                new ParallelCommandGroup(
                    new Elevatorsetpos(m_Elevator, elevatorL2Algaepos),
                    new FerrisWheelSetPos(m_FerrisWheel, ferrisAlgaeReefPic))));

    controller
        .povRight()
        .onTrue(
            new SequentialCommandGroup(
                new FerrisWheelVertical(m_FerrisWheel),
                new Elevatorsetpos(m_Elevator, elevatoridlepos),
                new ParallelCommandGroup(
                    new Elevatorsetpos(m_Elevator, elevatorL3Algaepos),
                    new FerrisWheelSetPos(m_FerrisWheel, ferrisAlgaeReefPic))));

    // controller
    //     .x()
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             new FerrisWheelVertical(m_FerrisWheel),
    //             new Elevatorsetpos(m_Elevator, elevatoridleposL2),
    //             new SequentialCommandGroup(
    //                 new FerrisWheelSetPos(m_FerrisWheel, ferriscoralplaceL2),
    //                 new Elevatorsetpos(m_Elevator, elevatorL2pos))));

    controller
        .a()
        .onTrue(
            new SequentialCommandGroup(
                new FerrisWheelVertical(m_FerrisWheel),
                new Elevatorsetpos(m_Elevator, elevatoridlepos),
                new SequentialCommandGroup(
                    new Elevatorsetpos(m_Elevator, elevatorRetreivepos),
                    new FerrisWheelSetPos(m_FerrisWheel, ferriscoralretreive))));

    testcontroller
        .leftStick()
        .onTrue(
            new ParallelRaceGroup(
                new FerrisWheelSetPos(m_FerrisWheel, ferriswheelvert),
                new Elevatorsetpos(m_Elevator, -5)));

    testcontroller
        .rightStick()
        .onTrue(
            new ParallelRaceGroup(
                new FerrisWheelSetPos(m_FerrisWheel, ferriswheelvert),
                new Elevatorsetpos(m_Elevator, -25)));

    // ferris wheel testing buttons
    testcontroller
        .povRight()
        .onTrue(
            new ParallelRaceGroup(
                new FerrisWheelSetPos(m_FerrisWheel, ferrisAlgaeReefPic),
                new Elevatorsetpos(m_Elevator, -25)));

    testcontroller
        .povLeft()
        .onTrue(
            new ParallelRaceGroup(
                new FerrisWheelSetPos(m_FerrisWheel, ferriscoralplace),
                new Elevatorsetpos(m_Elevator, -25)));

    testcontroller
        .x()
        .onTrue(new InstantCommand(m_coralground::runtoposition1)) // retract
        .onFalse(new InstantCommand(m_coralground::stop));

    testcontroller
        .b()
        .onTrue(new InstantCommand(m_coralground::runtoposition3)) // place coral on reef
        .onFalse(new InstantCommand(m_coralground::stop));

    testcontroller
        .a()
        .onTrue(new InstantCommand(m_coralground::runtoposition2)) // run to ground
        .onFalse(new InstantCommand(m_coralground::stop));

    testcontroller
        .y()
        .onTrue(new InstantCommand(m_coralground::homingroutine))
        .onFalse(new InstantCommand(m_coralground::stop));
    // controller
    //     .leftStick()
    //     .onTrue(new InstantCommand(m_FerrisWheel::placeposition))
    //     .onFalse(new InstantCommand(m_FerrisWheel::ferrisstop));
    // controller
    //     .rightStick()
    //     .onTrue(new InstantCommand(m_FerrisWheel::retreiveposition))
    //     .onFalse(new InstantCommand(m_FerrisWheel::ferrisstop));
    controller
        .back()
        .onTrue(new InstantCommand(m_FerrisWheel::algaeposition))
        .onFalse(new InstantCommand(m_FerrisWheel::ferrisstop));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
