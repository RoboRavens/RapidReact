package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.Conveyance.ConveyanceCollectCommand;
import frc.robot.commands.Feeder.FeederShootOneBallCommand;
import frc.robot.commands.Shooter.ShooterStartInstantCommand;
import frc.robot.commands.Shooter.ShooterStopCommand;
import frc.robot.commands.Shooter.ShooterTarmacCommand;
import frc.robot.commands.Shooter.ShooterWaitUntilIsRecoveredCommand;
import frc.util.PathWeaver;

public class FiveBallHps {
    public static Command get() {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/5 ball HPS-1.wpilib.json");
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/5 ball HPS-2.wpilib.json");

        var threeBall = ThreeBallTarmacAutoCommand.get();
        var moveToPlayerStationThenWait = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory1).andThen(new WaitCommand(3));
        var pickUpBallsFromPlayerStation = new ParallelDeadlineGroup(moveToPlayerStationThenWait, new ConveyanceCollectCommand());
        var moveToTarmacShot = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory2);

        var shootFourthAndFifthBalls = new ShooterWaitUntilIsRecoveredCommand()
            .andThen(new FeederShootOneBallCommand())
            .andThen(new ShooterWaitUntilIsRecoveredCommand())
            .andThen(new FeederShootOneBallCommand());

        return threeBall
            .andThen(pickUpBallsFromPlayerStation)
            .andThen(new ShooterTarmacCommand())
            .andThen(new ShooterStartInstantCommand())
            .andThen(moveToTarmacShot)
            .andThen(shootFourthAndFifthBalls)
            .andThen(new WaitCommand(.25))
            .andThen(new ShooterStopCommand());
    }
}
