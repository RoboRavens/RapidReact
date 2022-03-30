package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FeederEjectAllCommand extends CommandBase {
  public FeederEjectAllCommand() {
    addRequirements(Robot.FEEDER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.FEEDER_SUBSYSTEM.setFeederConveyanceEject();
    Robot.FEEDER_SUBSYSTEM.feederWheelReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.FEEDER_SUBSYSTEM.conveyanceStop();
    Robot.FEEDER_SUBSYSTEM.feederWheelStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
