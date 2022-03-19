package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Conveyance.ConveyanceCollectCommand;
import frc.robot.commands.Feeder.FeederForceShootDurationCommand;
import frc.robot.commands.Shooter.ShooterStartInstantCommand;
import frc.robot.commands.Shooter.ShooterStopCommand;
import frc.robot.commands.Shooter.ShooterTarmacCommand;
//import frc.robot.commands.shooter.ShooterWaitUntilIsRecoveredCommand;
import frc.util.PathWeaver;

public class TwoBallAutoCommand {
    public static Command getHangarCommand() {
        return TwoBallAutoCommand.get("hangar");
    }

    public static Command getWallCommand() {
        return TwoBallAutoCommand.get("wall");
    }

    public static Command getCenterCommand() {
        return new InstantCommand();
        // return TwoBallAutoCommand.get("center");
    }

    private static Command get(String type) {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/2 ball " + type + "-1.wpilib.json");

        var driveCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(trajectory1)
            .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory1))
            .andThen(new WaitCommand(1));

        var runConveyanceAndDriveToFirstBall = new ParallelDeadlineGroup(driveCommand, new ConveyanceCollectCommand());
        
        var shootBall = new FeederForceShootDurationCommand(Constants.AUTO_SHOOT_BALL_DURATION);
        
        /* new ShooterWaitUntilIsRecoveredCommand()
            .andThen(new FeederShootOneBallCommand())
            .andThen(new ShooterWaitUntilIsRecoveredCommand())
            .andThen(new FeederShootOneBallCommand());
            */

        return new ShooterTarmacCommand()
            .andThen(new ShooterStartInstantCommand())
            .andThen(runConveyanceAndDriveToFirstBall)
            .andThen(shootBall)
            .andThen(new WaitCommand(Constants.AUTO_WAIT_FOR_CONVEYANCE_DURATION))
            .andThen(shootBall)
            .andThen(new WaitCommand(.25))
            .andThen(new ShooterStopCommand());
    }
}
