package frc.robot.commands.DriveTrain;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveTrainDriveInchesCommand extends CommandBase {
    public DriveTrainDriveInchesCommand() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
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

        Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Robot.DRIVE_TRAIN_SUBSYSTEM.
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.DRIVE_TRAIN_SUBSYSTEM.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
