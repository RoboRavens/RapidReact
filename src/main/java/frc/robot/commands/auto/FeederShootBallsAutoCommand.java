package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class FeederShootBallsAutoCommand {
    public static Command get(int ballsToShoot) {
        Command commandChain = new InstantCommand();
        var FS = Robot.FEEDER_SUBSYSTEM;

        for (int i = 0; i < ballsToShoot; i++) {
            commandChain = commandChain
                .andThen(new WaitUntilCommand(() -> FS.getFeederHasBall()).withTimeout(.25))
                .andThen(new WaitUntilCommand(() -> Robot.SHOOTER_SUBSYSTEM.motorsAreRecovered()).withTimeout(.25))
                .andThen(new StartEndCommand(FS::forceShoot, FS::stopFeederAndConveyance, FS)
                    .until(() -> FS.getFeederHasBall() == false)
                    .withTimeout(.25)
                );
        }

        return commandChain
            .andThen(new WaitCommand(.25));
    }
}
