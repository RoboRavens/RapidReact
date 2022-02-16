package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ConveyanceIndexCommmand extends CommandBase {
    private boolean isBallBetweenSensors; // ball state
    
    public void ConveyanceIndexCommand() {
        addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceOneSubsystemHasBall() == true && Robot.FEEDER_SUBSYSTEM.getFeederSubsystemHasBall() == true) {
            Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();   //when there is a ball in conveyance stage 1 and 2 conveyance wont run        
        }        
    
        if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceOneSubsystemHasBall() == true && Robot.FEEDER_SUBSYSTEM.getFeederSubsystemHasBall() == false)              {        
         Robot.CONVEYANCE_SUBSYSTEM.setConveyanceMaxForward();  //if there is a ball in comveyance stage 1 but nothing at stage 2 conveyance will run at 1
       }
   
    }
        
     // Called once the command ends or is interrupted.
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