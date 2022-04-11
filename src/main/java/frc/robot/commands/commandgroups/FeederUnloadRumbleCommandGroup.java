package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.controls.ControllerRumbleCommand;

public class FeederUnloadRumbleCommandGroup extends SequentialCommandGroup {
    
    public FeederUnloadRumbleCommandGroup() {
        addCommands(
            new WaitCommand(.5),
            new ControllerRumbleCommand(.5)
        );
    }

}
