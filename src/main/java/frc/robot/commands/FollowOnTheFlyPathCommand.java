// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;
// import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.drive.Drive;
// import java.util.List;

// /* You should consider using the more terse Command factories API instead
// https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class FollowOnTheFlyPathCommand extends Command {

//   /** Creates a new FollowPathCommand. */
//   private final Drive drivetrain;

//   private final Translation2d targetPosition;
//   private PathPlannerTrajectory path;

//   public FollowOnTheFlyPathCommand(Drive drivetrain, Translation2d targetPosition) {
//     this.drivetrain = drivetrain;
//     this.targetPosition = targetPosition;
//     addRequirements(drivetrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//     // Get the current robot pose from the drivetrain
//     List<Waypoint> waypoints =
//         PathPlannerPath.waypointsFromPoses(
//             new Pose2d(
//                 drivetrain.getPose().getX(),
//                 drivetrain.getPose().getY(),
//                 drivetrain.getPose().getRotation()),
//             new Pose2d(3.103, 3.706, Rotation2d.fromDegrees(180)));

//     PathConstraints constraints =
//         new PathConstraints(2, 2, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

//     // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also
// use
//     // unlimited constraints, only limited by motor torque and nominal battery voltage

//     // Create the path using the waypoints created above
//     PathPlannerPath path =
//         new PathPlannerPath(
//             waypoints,
//             constraints,
//             null, // The ideal starting state, this is only relevant for pre-planned paths, so
// can
//             // be null for on-the-fly paths.
//             new GoalEndState(
//                 0.0,
//                 Rotation2d.fromDegrees(
//                     180)) // Goal end state. You can set a holonomic rotation here. If using a
//             // differential drivetrain, the rotation will have no effect.
//             );
//     // Prevent the path from being flipped if the coordinates are already correct
//     path.preventFlipping = true;

//     System.out.println("Following on-the-fly path to: ");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     drivetrain.followPath(path);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
