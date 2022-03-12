package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class ConveyanceIndexCommand extends CommandBase {
    private boolean isBallBetweenSensors; // ball state
    
    public ConveyanceIndexCommand() {
        addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
     if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall()  && Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == false ) {
         Robot.CONVEYANCE_SUBSYSTEM.setConveyanceMaxForward();   //when there is a ball in conveyance stage 1 and 2 conveyance wont run        
       } 
         else if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall()  && Robot.FEEDER_SUBSYSTEM.getFeederHasBall() )    {        
         Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();  //if there is a ball in comveyance stage 1 but nothing at stage 2 conveyance will run at 1
       } 
        else if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() == false ) {
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