package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.conveyance.ConveyanceCollectCommand;
import frc.robot.commands.conveyance.ConveyanceEjectCommand;
import frc.robot.commands.conveyance.ConveyanceIndexCommand;
import frc.robot.commands.feeder.FeederUnloadCommand;
import frc.robot.commands.shooter.ShooterLowGoalCommand;
import frc.robot.commands.shooter.ShooterStartInstantCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.robot.commands.shooter.ShooterTarmacCommand;
import frc.util.AutoMode;
import frc.util.PathWeaver;

public class TwoBallAutoCommand {
    public static AutoMode getHangarAutoMode() {
        return new AutoMode("Two Ball Hangar", TwoBallAutoCommand.get("hangar"));
    }

    public static AutoMode getHangarPlusOtherBallsHangarAutoMode(Command twoBallHangar) {
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/2 ball hangar-2.wpilib.json");
        var trajectory3 = PathWeaver.getTrajectoryFromFile("output/2 ball hangar-3.wpilib.json");

        var driveToMiddleOpposingTeamBall = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory2).andThen(new WaitCommand(1));
        var driveToTopOpposingTeamBall = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory3).andThen(new WaitCommand(1));
        var drive = new SequentialCommandGroup(driveToMiddleOpposingTeamBall, driveToTopOpposingTeamBall);
        var driveToOpposingTeamBallsWhilCollecting = new ParallelDeadlineGroup(drive, new ConveyanceCollectCommand());

        var cmd = twoBallHangar
            .andThen(new ShooterLowGoalCommand())
            .andThen(new ShooterStartInstantCommand())
            .andThen(new InstantCommand(() -> Robot.COLOR_SENSOR.setColorSensorFeatureEnabled(false))) // re-enabled in teleopInit
            .andThen(driveToOpposingTeamBallsWhilCollecting)
            .andThen(FeederShootBallsAutoCommand.get(2))
            .andThen(new ShooterStopCommand());

        return new AutoMode("Two Ball Hangar Plus Hangar", cmd);
    }

    public static AutoMode getHangarPlusOtherBallsByGoalAutoMode(Command twoBallHangar) {
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/2 ball hangar-2.wpilib.json");
        var trajectory3 = PathWeaver.getTrajectoryFromFile("output/2 ball hangar-3.wpilib.json");
        var trajectory4 = PathWeaver.getTrajectoryFromFile("output/2 ball hangar-4.wpilib.json");

        var driveToMiddleOpposingTeamBall = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory2).andThen(new WaitCommand(1));
        var driveToTopOpposingTeamBall = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory3).andThen(new WaitCommand(1));
        var drive = new SequentialCommandGroup(driveToMiddleOpposingTeamBall, driveToTopOpposingTeamBall);
        var driveToOpposingTeamBallsWhilCollecting = new ParallelDeadlineGroup(drive, new ConveyanceCollectCommand());

        var driveToEjectionPoint = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory4);

        var cmd = twoBallHangar
            .andThen(new InstantCommand(() -> Robot.COLOR_SENSOR.setColorSensorFeatureEnabled(false)))  // re-enabled in teleopInit
            .andThen(driveToOpposingTeamBallsWhilCollecting)
            .andThen(driveToEjectionPoint)
            .andThen(new ConveyanceEjectCommand().withTimeout(3))
            .andThen(new ShooterStopCommand());
        
        return new AutoMode("Two Ball Hangar Plus Goal", cmd);
    }

    public static AutoMode getWallAutoMode() {
        return new AutoMode("Two Ball Wall", TwoBallAutoCommand.get("wall"));
    }

    private static Command get(String type) {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/2 ball " + type + "-1.wpilib.json");

        var driveCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(trajectory1)
            .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.getMarkPositionCommand())
            .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory1))
            .andThen(new WaitCommand(1));

        var runConveyanceAndDriveToFirstBall = new ParallelDeadlineGroup(driveCommand, new ConveyanceCollectCommand());
        var unload = new FeederUnloadCommand().withTimeout(2);

        return new ShooterTarmacCommand()
            .andThen(new ShooterStartInstantCommand())
            .andThen(runConveyanceAndDriveToFirstBall)
            .andThen(new ParallelDeadlineGroup(unload, new ConveyanceIndexCommand()))
            .andThen(new ShooterStopCommand());
    }
}
