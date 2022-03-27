package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.conveyance.ConveyanceCollectCommand;
import frc.robot.commands.shooter.ShooterStartInstantCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.robot.commands.shooter.ShooterTarmacCommand;
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
            .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.getMarkPositionCommand())
            .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory1))
            .andThen(new WaitCommand(1));

        var runConveyanceAndDriveToFirstBall = new ParallelDeadlineGroup(driveCommand, new ConveyanceCollectCommand());

        var shootBallsOneAndTwo = FeederShootBallsAutoCommand.get(2);

        return new ShooterTarmacCommand()
            .andThen(new ShooterStartInstantCommand())
            .andThen(runConveyanceAndDriveToFirstBall)
            .andThen(shootBallsOneAndTwo)
            .andThen(new ShooterStopCommand());
    }
}
