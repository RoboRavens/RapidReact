package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.controls.ControllerRumbleCommand;

public class ControllerRumbleTwiceCommandGroup extends SequentialCommandGroup {

  public ControllerRumbleTwiceCommandGroup() {
    addCommands(
        new ControllerRumbleCommand(.25),
        new WaitCommand(.25),
        new ControllerRumbleCommand(.5)
    );
  }

}