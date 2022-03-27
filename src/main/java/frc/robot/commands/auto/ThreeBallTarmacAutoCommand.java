package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.conveyance.ConveyanceCollectCommand;
import frc.robot.commands.shooter.ShooterLaunchpadCommand;
import frc.robot.commands.shooter.ShooterStartInstantCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.util.PathWeaver;

public class ThreeBallTarmacAutoCommand {
    public static Command get() {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/3 ball tarmac-1.wpilib.json");

        var twoBall = TwoBallAutoCommand.getWallCommand();

        var driveThenWait = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory1).andThen(new WaitCommand(.5));
        var pickUpThirdBallWhileMovingToLaunchpadShot = new ParallelDeadlineGroup(driveThenWait, new ConveyanceCollectCommand());
        var shootThirdBall = FeederShootBallsAutoCommand.get(1);

        return twoBall
            .andThen(new ShooterLaunchpadCommand())
            .andThen(new ShooterStartInstantCommand())
            .andThen(pickUpThirdBallWhileMovingToLaunchpadShot)
            .andThen(shootThirdBall)
            .andThen(new ShooterStopCommand());
    }
}
