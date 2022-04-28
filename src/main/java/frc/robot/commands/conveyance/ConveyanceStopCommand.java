package frc.robot.commands.conveyance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ConveyanceStopCommand extends CommandBase {
  boolean conveyanceStopped = false;
  
  public ConveyanceStopCommand() {
    addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyanceStopped = false;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("conveyance stop command running");
    Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
    conveyanceStopped = true;
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(conveyanceStopped) {
      System.out.println("conveyance stop command is finished");
      return true;
    }
    return false;
  }
}
