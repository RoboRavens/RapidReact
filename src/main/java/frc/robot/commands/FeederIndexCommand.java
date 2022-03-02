package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FeederIndexCommand extends CommandBase {
    private boolean isBallBetweenSensors; // ball state
    
    public FeederIndexCommand() {
        addRequirements(Robot.FEEDER_SUBSYSTEM);
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall());
        System.out.println(Robot.FEEDER_SUBSYSTEM.getFeederHasBall());

        // (False means no ball in front of the sensor and vice versa)
        
        // If there is a ball in conveyance stage 1 and no ball in conveyance stage 2 (feeder subsystem),
        // run the conveyance stage 2 and set isBallBetweenSensors to true
        if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() == true && Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == false) {
            Robot.FEEDER_SUBSYSTEM.setConveyanceNormalSpeedForward();
            isBallBetweenSensors = true;
            System.out.println("Feeder wheel motor running");
        }
        // If the ball state is in between sensors on the conveyance,
        // cotninue to run conveyance stage 2
        if(isBallBetweenSensors == true) {
            Robot.FEEDER_SUBSYSTEM.setConveyanceNormalSpeedForward();
        }
        // If there is a ball completely within conveyance stage 2,
        // set isBallBetweenSensors to false and stop conveyance stage 2
        if(Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == true) {
            isBallBetweenSensors = false;
            Robot.FEEDER_SUBSYSTEM.stopConveyance();
        }
        // If there are no balls in the robot,
        // set isBallBetweenSensors to false and stop conveyance stage 2
        if((Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() == false) && (Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == false)) {
            isBallBetweenSensors = false;
            Robot.FEEDER_SUBSYSTEM.stopConveyance();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.FEEDER_SUBSYSTEM.stopConveyance();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
