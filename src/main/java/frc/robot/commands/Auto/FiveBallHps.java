package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.ConveyanceCollectCommand;
import frc.robot.commands.FeederShootOneBallCommand;
import frc.robot.commands.shooter.ShooterLaunchpadCommand;
import frc.robot.commands.shooter.ShooterStartCommand;
import frc.robot.commands.shooter.ShooterStartInstantCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.robot.commands.shooter.ShooterWaitUntilIsRecoveredCommand;
import frc.util.PathWeaver;

public class FiveBallHps {
    public static Command get() {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/5 ball HPS-1.wpilib.json");
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/5 ball HPS-2.wpilib.json");
        var trajectory3 = PathWeaver.getTrajectoryFromFile("output/5 ball HPS-3.wpilib.json");

        var everythingUpToThirdBall = ThreeBallTarmacAutoCommand.getPartialCommandWhichPicksUpThirdBall();
        var moveToLaunchpadShotFromThirdBall = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory1);
        var shootThirdBall = new ShooterWaitUntilIsRecoveredCommand()
            .andThen(new FeederShootOneBallCommand());

        var moveToPlayerStationAndWait = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory2).andThen(new WaitCommand(3));
        var moveToPlayerStationWhileCollectingAndWait = new ParallelDeadlineGroup(moveToPlayerStationAndWait, new ConveyanceCollectCommand());

        var moveToLaunchpadShotFromPlayerStation = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory3);

        var shootFourthAndFifthBalls = new ShooterWaitUntilIsRecoveredCommand()
            .andThen(new FeederShootOneBallCommand())
            .andThen(new ShooterWaitUntilIsRecoveredCommand())
            .andThen(new FeederShootOneBallCommand());

        return everythingUpToThirdBall
            .andThen(new ShooterLaunchpadCommand())
            .andThen(new ShooterStartInstantCommand())
            .andThen(moveToLaunchpadShotFromThirdBall)
            .andThen(shootThirdBall)
            .andThen(moveToPlayerStationWhileCollectingAndWait)
            .andThen(moveToLaunchpadShotFromPlayerStation)
            .andThen(shootFourthAndFifthBalls)
            .andThen(new ShooterStopCommand());
    }
}
