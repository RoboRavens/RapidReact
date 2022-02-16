package frc.robot.commands.DriveTrain;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class DriveTrainTrajectories {
    public static Command driveStraightInches(){
        return new InstantCommand();
    }

    public static Command sCurveDemo() {
        var trajectoryConfig = Robot.DRIVE_TRAIN_SUBSYSTEM.GetTrajectoryConfig();
        
        // An example trajectory to follow.  All units in meters.
        var trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            trajectoryConfig);

        return Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory);
    }

    public static Command driveStraightOneMeter() {
        var trajectoryConfig = Robot.DRIVE_TRAIN_SUBSYSTEM.GetTrajectoryConfig();
        var trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(.5, 0)),
            new Pose2d(1, 0, new Rotation2d(0)),
            trajectoryConfig);

        return Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory);
    }

    public static Command moveAndRotate(double distanceInMeters, double degrees) {
        var trajectoryConfig = Robot.DRIVE_TRAIN_SUBSYSTEM.GetTrajectoryConfig();
        var trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(distanceInMeters, 0, Rotation2d.fromDegrees(degrees)),
            trajectoryConfig);

        return Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory);
    }
}
