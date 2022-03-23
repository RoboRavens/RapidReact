package frc.robot.commands.Conveyance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ConveyanceSubsystem;

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
        Robot.CONVEYANCE_SUBSYSTEM.setConveyanceIndexSpeedForward();   //when there is a ball in conveyance stage 1 and 2 conveyance wont run        
      } 
      else if (conveyanceHasBall == false ) {
        Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
      }
    
      if (Robot.CONVEYANCE_SUBSYSTEM.robotHas2Balls() == true) {
       Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne(); 
       Robot.INTAKE_SUBSYSTEM.retract();
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