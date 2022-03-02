package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.FeederSafetyReverseCommand;

public class ShooterRevCommandgroup extends SequentialCommandGroup{

    public ShooterRevCommandgroup() {
        addCommands(
            Robot.FeederSafetyReverse,
            Robot.SHOOTER_START_COMMAND
        );
      }
    
}
