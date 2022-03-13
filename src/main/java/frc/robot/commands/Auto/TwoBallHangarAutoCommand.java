package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.util.PathWeaver;

public class TwoBallHangarAutoCommand {
    public static Command get() {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/2 ball hangar-1.wpilib.json");
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/2 ball hangar-2.wpilib.json");

        return new InstantCommand(() -> System.out.println("start intake"))
        .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(trajectory1))
        .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory1))
        .andThen(new InstantCommand(() -> System.out.println("stop intake")))
        .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommand(trajectory2))
        .andThen(new InstantCommand(() -> System.out.println("shoot")));
    }
}
