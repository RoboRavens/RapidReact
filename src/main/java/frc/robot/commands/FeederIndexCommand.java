package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FeederIndexCommand extends CommandBase {
    
    public FeederIndexCommand() {
        addRequirements(Robot.FEEDER_SUBSYSTEM);
        addRequirements(Robot.INTAKE_SUBSYSTEM);
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Stop the conveyance subsystems if there are two balls in the robot
        if((Robot.FEEDER_SUBSYSTEM.getFeederCargoSensor() == false) && (Robot.FEEDER_SUBSYSTEM.getFeederCargoSensor() == false)) {
            Robot.FEEDER_SUBSYSTEM.stopConveyance();
            Robot.INTAKE_SUBSYSTEM.stop();
        }
        // Run the intake (zone 1) if there is one ball in zone 1 of the conveyor. This moves the ball to the feeder (zone 2)
        if((Robot.FEEDER_SUBSYSTEM.getFeederCargoSensor() == true) && (Robot.FEEDER_SUBSYSTEM.getFeederCargoSensor() == false)) {
            Robot.FEEDER_SUBSYSTEM.stopConveyance();
            Robot.INTAKE_SUBSYSTEM.collect();
        }
        // __ if there are zero balls in the robot
        if((Robot.FEEDER_SUBSYSTEM.getFeederCargoSensor() == true) && (Robot.FEEDER_SUBSYSTEM.getFeederCargoSensor() == true)) {
            
        }

        if((Robot.FEEDER_SUBSYSTEM.getFeederCargoSensor() == false) && (Robot.FEEDER_SUBSYSTEM.getFeederCargoSensor() == true)) {

        }
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
