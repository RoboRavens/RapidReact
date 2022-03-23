package frc.robot.commands.Conveyance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ConveyanceIndexCommand extends CommandBase {

    private boolean firstSensorHadBall = false;

    public ConveyanceIndexCommand() {
        addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      boolean firstSensorHasBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBallNewFirstSensor();
      boolean conveyanceHasBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall();
      boolean feederHasBall = Robot.FEEDER_SUBSYSTEM.getFeederHasBall();
      boolean conveyanceColorIsCorrect = false; // Should be replaced by a method call to get the sensor input
      boolean feederColorIsCorrect = false; // Should be replaced by a method call to get the sensor input
      boolean atLeastOneBallInConveyanceOne = false;
      boolean onlyOneBallInConveyance = false;

      if (firstSensorHasBall || conveyanceHasBall) {
        atLeastOneBallInConveyanceOne = true;
      }
      if (atLeastOneBallInConveyanceOne && feederHasBall == false) {
        onlyOneBallInConveyance = true;
      }

      if (conveyanceColorIsCorrect && feederColorIsCorrect) {
        if (onlyOneBallInConveyance || firstSensorHadBall) {
          Robot.CONVEYANCE_SUBSYSTEM.setConveyanceIndexSpeedForward();   //when there is a ball in conveyance stage 1 and 2 conveyance wont run      
          firstSensorHadBall = true;
        } 
        else if (conveyanceHasBall && feederHasBall) {        
          Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();  //if there is a ball in comveyance stage 1 but nothing at stage 2 conveyance will run at 1
          firstSensorHadBall = false;
        } 
        else if (atLeastOneBallInConveyanceOne == false) {
          Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
          firstSensorHadBall = false;
        }
      }
      else if (conveyanceColorIsCorrect == false && feederHasBall) {
        Robot.CONVEYANCE_SUBSYSTEM.setConveyanceNormalSpeedReverse();
      }
      else if (conveyanceColorIsCorrect == false && feederHasBall == false) {
        Robot.CONVEYANCE_SUBSYSTEM.setConveyanceIndexSpeedForward();
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