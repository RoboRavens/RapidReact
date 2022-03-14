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

public class ThreeBallTarmacAutoCommand {
    public static Command getPartialCommandWhichPicksUpThirdBall() {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/3 ball tarmac-1.wpilib.json");
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/3 ball tarmac-2.wpilib.json");
        var trajectory3 = PathWeaver.getTrajectoryFromFile("output/3 ball tarmac-3.wpilib.json");

        var pickUpSecondBall = new ParallelDeadlineGroup(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(trajectory1))
                .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory1)
                .andThen(new WaitCommand(1)),
            new ConveyanceCollectCommand()
        );

        var moveForTarmacShot = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory2);

        var shootFirstTwoBalls = new ShooterWaitUntilIsRecoveredCommand()
            .andThen(new FeederShootOneBallCommand())
            .andThen(new ShooterWaitUntilIsRecoveredCommand())
            .andThen(new FeederShootOneBallCommand());

        var pickUpThirdBall = new ParallelDeadlineGroup(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory3)
                .andThen(new WaitCommand(1)),
            new ConveyanceCollectCommand()
        );

        var everything = new ShooterTarmacCommand()
            .andThen(pickUpSecondBall)
            .andThen(moveForTarmacShot)
            .andThen(shootFirstTwoBalls)
            .andThen(pickUpThirdBall);

        return everything;
    }

    public static Command get() {
        var trajectory4 = PathWeaver.getTrajectoryFromFile("output/3 ball tarmac-4.wpilib.json");

        var everythingUpToThirdBall = ThreeBallTarmacAutoCommand.getPartialCommandWhichPicksUpThirdBall();
        var moveToTarmacShot = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory4);
        var shootThirdBall = new ShooterWaitUntilIsRecoveredCommand()
            .andThen(new FeederShootOneBallCommand());

        var everything = everythingUpToThirdBall
            .andThen(moveToTarmacShot)
            .andThen(shootThirdBall);

        return new ParallelDeadlineGroup(everything, new ShooterStartCommand());
    }
}
