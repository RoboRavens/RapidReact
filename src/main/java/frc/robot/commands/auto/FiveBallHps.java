package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.conveyance.ConveyanceCollectCommand;
import frc.robot.commands.conveyance.ConveyanceIndexCommand;
import frc.robot.commands.feeder.FeederIndexCommand;
import frc.robot.commands.feeder.FeederUnloadCommand;
import frc.robot.commands.shooter.ShooterLaunchpadCommand;
import frc.robot.commands.shooter.ShooterStartInstantCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.util.AutoMode;
import frc.util.PathWeaver;

public class FiveBallHps {
    public static AutoMode getAutoMode(Command threeBallTarmac) {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/5 ball HPS-1.wpilib.json");
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/5 ball HPS-2.wpilib.json");

        var moveToPlayerStationThenWait = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory1).andThen(new WaitCommand(1));
        var moveToTarmacShot = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory2);
        var moveToPlayerStationAndBackToTarmacWhileCollecting = moveToPlayerStationThenWait.andThen(moveToTarmacShot);
        var pickUpBallsFromPlayerStation = new ParallelDeadlineGroup(moveToPlayerStationAndBackToTarmacWhileCollecting, new ConveyanceCollectCommand());

        var cmd = threeBallTarmac
            .andThen(new ShooterLaunchpadCommand())
            .andThen(new ShooterStartInstantCommand())
            .andThen(new ParallelDeadlineGroup(pickUpBallsFromPlayerStation, new FeederIndexCommand()))
            .andThen(new ParallelDeadlineGroup(new FeederUnloadCommand(), new ConveyanceIndexCommand()))
            .andThen(new ShooterStopCommand());
        
        return new AutoMode("Five Ball HPS", cmd);
    }
}
