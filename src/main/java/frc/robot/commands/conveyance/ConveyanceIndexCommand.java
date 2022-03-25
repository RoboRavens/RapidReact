package frc.robot.commands.conveyance;
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
      boolean entranceBeamBreakHasBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceIntakeBeamBreakHasBall();
      boolean stagingBeamBreakHasBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall();
      boolean feederHasBall = Robot.FEEDER_SUBSYSTEM.getFeederHasBall();
      boolean atLeastOneBallInConveyanceOne = false;
      boolean onlyOneBallInConveyance = false;

      if (entranceBeamBreakHasBall || stagingBeamBreakHasBall) {
        atLeastOneBallInConveyanceOne = true;
      }
      if (atLeastOneBallInConveyanceOne && feederHasBall == false) {
        onlyOneBallInConveyance = true;
      }

      if (onlyOneBallInConveyance || firstSensorHadBall) {
        Robot.CONVEYANCE_SUBSYSTEM.setConveyanceIndexSpeedForward();   //when there is a ball in conveyance stage 1 and 2 conveyance wont run      
        firstSensorHadBall = true;
      } 
      else if (stagingBeamBreakHasBall && feederHasBall) {        
        Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();  //if there is a ball in comveyance stage 1 but nothing at stage 2 conveyance will run at 1
        firstSensorHadBall = false;
      } 
      else if (atLeastOneBallInConveyanceOne == false) {
        Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
        firstSensorHadBall = false;
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


    /*
    ADDITIONAL UN-IMPLEMENTED CONVEYANCE INDEX LOGIC

    Conveyance stage one:
    - If feeder has a ball:
      - If conveyance staging beam break sees a ball
        - If staging beam break ball is proper color
          - If it is, and conveyance entry does NOT see a ball - done
          - If it is, but conveyance entry DOES see a ball - eject the ball until conveyance entry doesn't see anything, and then run the conveyance forward again until the staging beam break sees the ball again
        - If staging beam break ball is NOT proper color
          - Eject until the entry beam break shows true (ball is passing through) and then false (ball has finished passing through) and then done
      - If conveyance staging does NOT see a ball
        - If conveyance entry sees a ball:
          - Move the ball up to the staging bream break (similar to how the feeder waits for a ball from the conveyance - the conveyance staging bream break can "wait" for a ball from the conveyance entry beam break)
          - Once the staging beam break sees the ball, it will be covered by the conditions above that check the color of the ball and eject if necessary
        - If conveyance entry does NOT see a ball (and did not recently see a ball such that staging is "waiting"): there are no balls in the stage 1 conveyance, so nothing needs to be done
    - If the feeder does NOT have a ball:
      - Run the intake and conveyance like normal; the first ball collected should go right to the feeder regardless of its color.
      - The one change to make here is if the ENTRY beam break sees a ball, it needs to index it to the staging beam break (which will then index it to the feeder.)
    
    */
}