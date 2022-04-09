package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.conveyance.ConveyanceCollectCommand;
import frc.robot.commands.conveyance.ConveyanceIndexCommand;
import frc.robot.commands.feeder.FeederIndexCommand;
import frc.robot.commands.feeder.FeederUnloadCommand;
import frc.robot.commands.shooter.ShooterAutoRadiusCommand;
import frc.robot.commands.shooter.ShooterLaunchpadCommand;
import frc.robot.commands.shooter.ShooterStartInstantCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.util.AutoMode;
import frc.util.PathWeaver;

public class ThreeBallTarmacAutoCommand {
    public static AutoMode getAutoMode() {
        var twoBallWall = TwoBallAutoCommand.getWallAutoMode().getAutoCommand();
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/3 ball tarmac-1.wpilib.json");
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/3 ball tarmac-2.wpilib.json");

        var drive = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory1)
            .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory2));
        var pickUpThirdBallWhileMovingToLaunchpadShot = new ParallelDeadlineGroup(drive, new ConveyanceCollectCommand());
        var unload = new FeederUnloadCommand().withTimeout(2);
        
        var cmd = twoBallWall
            .andThen(new ShooterLaunchpadCommand())
            .andThen(new ShooterStartInstantCommand())
            .andThen(new ParallelDeadlineGroup(pickUpThirdBallWhileMovingToLaunchpadShot, new FeederIndexCommand()))
            .andThen(new ParallelDeadlineGroup(unload, new ConveyanceIndexCommand()))
            .andThen(new ShooterStopCommand());

        return new AutoMode("Three Ball Tarmac", cmd);
    }

    public static AutoMode getTurretAutoMode() {
        var twoBallWall = TwoBallAutoCommand.getWallAutoMode().getAutoCommand();
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/3 ball tarmac turret-1.wpilib.json");

        var drive = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory1);
        var pickUpThirdBallWhileMovingToLaunchpadShot = new ParallelDeadlineGroup(drive, new ConveyanceCollectCommand());
        var unload = new FeederUnloadCommand().withTimeout(2);
        
        var cmd = twoBallWall
            .andThen(new ShooterLaunchpadCommand())
            .andThen(new ShooterStartInstantCommand())
            .andThen(new ParallelDeadlineGroup(pickUpThirdBallWhileMovingToLaunchpadShot, new FeederIndexCommand()))
            .andThen(new ParallelDeadlineGroup(unload, new ConveyanceIndexCommand()))
            .andThen(new ShooterStopCommand());

        return new AutoMode("Three Ball Tarmac Turret", cmd);
    }
}
