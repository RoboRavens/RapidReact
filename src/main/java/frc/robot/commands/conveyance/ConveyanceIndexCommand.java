package frc.robot.commands.conveyance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class ConveyanceIndexCommand extends CommandBase {
    public ConveyanceIndexCommand() {
        addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      System.out.println("conveyance index command running");
      boolean entranceBeamBreakHasBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceEntryBeamBreakHasBall();
      boolean stagingBeamBreakHasBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall();
      boolean feederHasBall = Robot.FEEDER_SUBSYSTEM.getFeederHasBall();
      boolean ballIsEjecting = Robot.CONVEYANCE_SUBSYSTEM.getIsBallEjecting();
      boolean conveyanceEjectingWrongColorCargo = Robot.CONVEYANCE_SUBSYSTEM.getIsConveyanceEjectingWrongColorCargo();
      boolean conveyanceEjectingThirdBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceEjectingThirdBall();
      boolean stagingEjectionPassThroughIsOccurring = Robot.CONVEYANCE_SUBSYSTEM.getStagingEjectionPassThroughIsOccurring();

      // new InstantCommand().until(() -> feederHasBall)
      //   .andThen(new InstantCommand().until(stagingBeamBreakHasBall));

      if (feederHasBall) {
        if (stagingBeamBreakHasBall && ballIsEjecting == false) {
          // Both feeder and staging have a ball, and we're not in an eject sequence.
          Robot.CONVEYANCE_SUBSYSTEM.setIsIndexingFromEntranceToStaging(false);
          if (Robot.CONVEYANCE_SUBSYSTEM.conveyanceHasProperColorCargo()) {
            if (entranceBeamBreakHasBall == false) {
              // If the staging ball is the correct color and there's no third ball at the entrance,
              // nothing further needs to be done.
              // Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
              ConveyanceStopCommand conveyanceStop = new ConveyanceStopCommand();
              conveyanceStop.schedule();
            }
            else if (entranceBeamBreakHasBall) { 
              // If there is a third ball in the entrance, begin the eject sequence
              ConveyanceEjectThirdBallCommand conveyanceEjectThirdBallCommand = new ConveyanceEjectThirdBallCommand();
              conveyanceEjectThirdBallCommand.schedule();
            }
          }
          else if (Robot.CONVEYANCE_SUBSYSTEM.conveyanceHasWrongColorCargo()) {
            // If the staged ball is the wrong color, and the feeder has a ball,
            // then the staged ball needs to be ejected by the conveyance.
            // Robot.CONVEYANCE_SUBSYSTEM.setConveyanceEjectCargo();
            // ballIsEjecting = true;
            // conveyanceEjectingWrongColorCargo = true;
            ConveyanceEjectCommand conveyanceEject = new ConveyanceEjectCommand();
            conveyanceEject.schedule();
          }
        }
        else if (stagingBeamBreakHasBall == false && ballIsEjecting == false) {
          // If there's no ball in the staging area, AND we're also not in the process of ejecting one.
          if (entranceBeamBreakHasBall || Robot.CONVEYANCE_SUBSYSTEM.getIsIndexingFromEntranceToStaging()) {
            // If there is, or was, a ball in the entrance beam break, it needs to be indexed.
            Robot.CONVEYANCE_SUBSYSTEM.setConveyanceIndexCargoForward();
            Robot.CONVEYANCE_SUBSYSTEM.setIsIndexingFromEntranceToStaging(true);
          }
          else {
            // Else, there neither is nor was a ball in the entrance, so nothing needs to be done.
            // Any scenario involving a ball in the feeder but no other ball in the robot
            // is handled solely by the feeder subsystem.
            Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
          }
        }
        else if (ballIsEjecting) {
          // Conveyance ejection sequence. This occurs when we pick up a third ball, OR a ball of the wrong
          // color, but only when the feeder already has a ball (so we can't eject through the shooter.)
          if (conveyanceEjectingWrongColorCargo) {
            // If a ball is ejecting because it's the wrong color.
            Robot.CONVEYANCE_SUBSYSTEM.setConveyanceEjectCargo();
            if (entranceBeamBreakHasBall) {
              // If there is a ball in the entrance beam break, the ejection pass-through is underway.
              // Either way the ejection continues, so there's no additional robot action from this condition.
              stagingEjectionPassThroughIsOccurring = true;
            }
            else {
              // There is no ball in the beam break - either the pass-through finished, or hasn't started yet.
              // There's no else statement here because if the beam break does not see a ball,
              // and ejection is NOT occurring, that just means the ball is in transit from staging to entrance.
              // The conveyance is already ejecting so no further action is necessary.
              if (stagingEjectionPassThroughIsOccurring) {
                // If the staging ejection pass-through was occurring, and the entrance bream break now reads false,
                // then the ejection pass-through has finished occurring.
                // That means the ejection is complete - stop the conveyance and reset the pass-through variable.
                stagingEjectionPassThroughIsOccurring = false;
                Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
                ballIsEjecting = false;
                conveyanceEjectingWrongColorCargo = false;
                Robot.CONVEYANCE_SUBSYSTEM.setIsIndexingFromEntranceToStaging(false);
              }
            }
          }
          else if (conveyanceEjectingThirdBall) {
            // If a ball is ejecting because there is a third ball in the robot.
            // This branch skips the pass-through logic of the wrong ball because the ball will be ejected
            // directly from the entrance beam beak before it ends up at the staging sensor.
            if (entranceBeamBreakHasBall == false) {
              Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
              ballIsEjecting = false;
              conveyanceEjectingThirdBall = false;
            }
          }
        }
      }
      else if (feederHasBall == false) {
        if (Robot.getRobotCargoInventory() == 0) {
          Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
        }
        else if (stagingBeamBreakHasBall) {
          Robot.CONVEYANCE_SUBSYSTEM.setConveyanceIndexCargoForward();
          Robot.CONVEYANCE_SUBSYSTEM.setIsIndexingFromEntranceToStaging(false);
        }
        else if (Robot.CONVEYANCE_SUBSYSTEM.getIsIndexingFromEntranceToStaging() || entranceBeamBreakHasBall) {
          Robot.CONVEYANCE_SUBSYSTEM.setConveyanceIndexCargoForward();
          Robot.CONVEYANCE_SUBSYSTEM.setIsIndexingFromEntranceToStaging(true);
        }
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
    Conveyance index logic overview

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