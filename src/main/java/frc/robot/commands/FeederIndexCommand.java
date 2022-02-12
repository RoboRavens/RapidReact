package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FeederIndexCommand extends CommandBase {
    private boolean isBallInBetweenSensors; // ball state
    
    public FeederIndexCommand() {
        addRequirements(Robot.FEEDER_SUBSYSTEM);
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // (True means no ball in front of the sensor and vice versa)
        
        // If there is a ball in conveyance stage 1 and no ball in conveyance stage 2 (feeder subsystem),
        // run the conveyance stage 2 and set isBallInBetweenSensors to true
        
        // If the ball state is in between sensors on the conveyance,
        // cotninue to run conveyance stage 2

        // If there is a ball completely within conveyance stage 2,
        // set isBallInBetweenSensors to false and stop conveyance stage 2

        // If there are no balls in the robot,
        // set isBallInBetweenSensors to false and stop conveyance stage 2
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
