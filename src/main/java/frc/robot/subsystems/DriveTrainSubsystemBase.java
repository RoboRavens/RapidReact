package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveTrainSubsystemBase extends SubsystemBase {
    public abstract void drive(ChassisSpeeds chassisSpeeds);
    public abstract Rotation2d getOdometryRotation();
    public abstract void holdPosition();
    public abstract void zeroGyroscope();
    public abstract void cutPower();
    public abstract void stopCutPower();
    public abstract TrajectoryConfig GetTrajectoryConfig();
    public abstract Command CreateSetOdometryToTrajectoryInitialPositionCommand(Trajectory trajectory);
    public abstract Command CreateFollowTrajectoryCommand(Trajectory trajectory);
    public abstract Command CreateFollowTrajectoryCommandSwerveOptimized(Trajectory trajectory);
}
