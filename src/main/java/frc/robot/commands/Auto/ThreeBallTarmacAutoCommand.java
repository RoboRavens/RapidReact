package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
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

public class ThreeBallTarmacAutoCommand {
    public static Command get() {
        var trajectory1 = PathWeaver.getTrajectoryFromFile("output/3 ball tarmac-1.wpilib.json");
        var trajectory2 = PathWeaver.getTrajectoryFromFile("output/3 ball tarmac-2.wpilib.json");

        var twoBall = TwoBallAutoCommand.getWallCommand();

        var driveThenWait = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory1).andThen(new WaitCommand(1));
        var waitThenConveyance = new WaitCommand(1).andThen(new ConveyanceCollectCommand());
        var pickUpThirdBall = new ParallelDeadlineGroup(driveThenWait, waitThenConveyance);

        var moveToTarmacShot = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory2);
        var shootBall = new FeederForceShootDurationCommand(Constants.AUTO_SHOOT_BALL_DURATION);
        
        /*new ShooterWaitUntilIsRecoveredCommand()
            .andThen(new FeederShootOneBallCommand());
            */

        return twoBall
            .andThen(new ShooterTarmacCommand())
            .andThen(new ShooterStartInstantCommand())
            .andThen(pickUpThirdBall)
            .andThen(moveToTarmacShot)
            .andThen(shootBall)
            .andThen(new WaitCommand(Constants.AUTO_WAIT_FOR_CONVEYANCE_DURATION))
            .andThen(shootBall)
            .andThen(new WaitCommand(.25))
            .andThen(new ShooterStopCommand());
    }
}
