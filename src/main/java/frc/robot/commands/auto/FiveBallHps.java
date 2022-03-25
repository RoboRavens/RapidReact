package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.conveyance.ConveyanceCollectCommand;
import frc.robot.commands.feeder.FeederForceShootDurationCommand;
import frc.robot.commands.shooter.ShooterStartInstantCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.robot.commands.shooter.ShooterTarmacCommand;
import frc.util.PathWeaver;

public class FiveBallHps {
    public static Command get() {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/5 ball HPS-1.wpilib.json");
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/5 ball HPS-2.wpilib.json");

        var threeBall = ThreeBallTarmacAutoCommand.get();
        var moveToPlayerStationThenWait = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory1).andThen(new WaitCommand(3));
        var pickUpBallsFromPlayerStation = new ParallelDeadlineGroup(moveToPlayerStationThenWait, new ConveyanceCollectCommand());
        var moveToTarmacShot = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory2);

        var shootBallsOneAndTwo = new FeederForceShootDurationCommand(Constants.TWO_BALL_SHOOTER_DURATION);
        /*var shootFourthAndFifthBalls = new ShooterWaitUntilIsRecoveredCommand()
            .andThen(new FeederShootOneBallCommand())
            .andThen(new ShooterWaitUntilIsRecoveredCommand())
            .andThen(new FeederShootOneBallCommand());*/

        return threeBall
            .andThen(pickUpBallsFromPlayerStation)
            .andThen(new ShooterTarmacCommand())
            .andThen(new ShooterStartInstantCommand())
            .andThen(moveToTarmacShot)
            .andThen(shootBallsOneAndTwo)
            // .andThen(new WaitCommand(.25))
            .andThen(new ShooterStopCommand());
    }
}