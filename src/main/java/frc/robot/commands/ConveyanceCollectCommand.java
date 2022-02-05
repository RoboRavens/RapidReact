package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ConveyanceCollectCommand extends CommandBase {

  public ConveyanceCollectCommand() {
    addRequirements(Robot.CONVEYANCE_COLLECT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ConveyanceCollectCommand init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("CONVEYANCE COLLECTING!!!");
    Robot.CONVEYANCE_COLLECT.setConveyanceMaxForward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

