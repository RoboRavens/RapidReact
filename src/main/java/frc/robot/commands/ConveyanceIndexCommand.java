package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ConveyanceIndexCommand extends CommandBase {
    public ConveyanceIndexCommand() {
        addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      boolean conveyanceHasBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall();
      boolean feederHasBall = Robot.FEEDER_SUBSYSTEM.getFeederHasBall();

      if (conveyanceHasBall && feederHasBall == false ) {
        Robot.CONVEYANCE_SUBSYSTEM.setConveyanceMaxForward();   //when there is a ball in conveyance stage 1 and 2 conveyance wont run        
      } 
      else if (conveyanceHasBall && feederHasBall) {        
        Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();  //if there is a ball in comveyance stage 1 but nothing at stage 2 conveyance will run at 1
        Robot.INTAKE_SUBSYSTEM.retract();
      } 
      else if (conveyanceHasBall == false ) {
        Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
      }
   }     
    
    @Override
    public void end(boolean interrupted) {
        Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}