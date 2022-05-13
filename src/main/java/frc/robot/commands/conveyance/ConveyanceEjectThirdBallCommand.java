package frc.robot.commands.conveyance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ConveyanceEjectThirdBallCommand extends CommandBase {
    boolean commandFinished = false;
  
  public ConveyanceEjectThirdBallCommand() {
    addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.CONVEYANCE_SUBSYSTEM.setConveyanceEjectCargo();
    Robot.CONVEYANCE_SUBSYSTEM.setIsBallEjecting(true);
    Robot.CONVEYANCE_SUBSYSTEM.setConveyanceEjectingThirdBall(true);
    Robot.CONVEYANCE_SUBSYSTEM.setStagingEjectionPassThroughIsOccurring(false);
    commandFinished = true;
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(commandFinished) {
        return true;
    }
    return false;
  }
}
