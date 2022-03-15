package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.ConveyanceCollectCommand;
import frc.robot.commands.FeederShootOneBallCommand;
import frc.robot.commands.shooter.ShooterStartCommand;
import frc.robot.commands.shooter.ShooterTarmacCommand;
import frc.robot.commands.shooter.ShooterWaitUntilIsRecoveredCommand;
import frc.util.PathWeaver;

public class TwoBallHangarAutoCommand {
    public static Command get() {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/2 ball hangar-1.wpilib.json");
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/2 ball hangar-2.wpilib.json");

        var runConveyanceAndDriveToFirstBall = new ParallelDeadlineGroup(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(trajectory1))
                .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory1)
                .andThen(new WaitCommand(1)),
            new ConveyanceCollectCommand()
        );

        var firstBallTurnAndShoot = runConveyanceAndDriveToFirstBall
            .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory2))
            .andThen(new ShooterWaitUntilIsRecoveredCommand())
            .andThen(new FeederShootOneBallCommand())
            .andThen(new ShooterWaitUntilIsRecoveredCommand())
            .andThen(new FeederShootOneBallCommand());

        return new ShooterTarmacCommand()
            .andThen(new ParallelDeadlineGroup(firstBallTurnAndShoot, new ShooterStartCommand()));
    }
}
