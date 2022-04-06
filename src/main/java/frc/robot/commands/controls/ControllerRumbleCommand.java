package frc.robot.commands.controls;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class ControllerRumbleCommand extends WaitCommand {
    public ControllerRumbleCommand() {
        super(0.25);
    }

    @Override
    public void initialize() {
      super.initialize();
      Robot.GAMEPAD.setRumbleOn();
    }

    @Override
    public void end(boolean interrupted) {
      super.end(interrupted);
      Robot.GAMEPAD.setRumbleOff();
    }
}
