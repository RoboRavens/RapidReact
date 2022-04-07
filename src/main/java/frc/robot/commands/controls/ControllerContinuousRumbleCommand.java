package frc.robot.commands.controls;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ControllerContinuousRumbleCommand extends CommandBase{
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
      Robot.GAMEPAD.setRumbleOn();
    }

    @Override
    public void end(boolean interrupted) {
      Robot.GAMEPAD.setRumbleOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
