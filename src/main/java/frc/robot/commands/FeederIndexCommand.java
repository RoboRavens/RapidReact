package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FeederIndexCommand extends CommandBase {
    private boolean isBallBetweenSensors; // ball state
    private boolean isRobotEmpty;
    
    public FeederIndexCommand() {
        addRequirements(Robot.FEEDER_SUBSYSTEM);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // False means no ball in front of the sensor and vice versa
        // For current testing purposes, pressing the button changes the value to false and vice versa
        // (i.e. holding both buttons means the robot is empty 
        // and leaving both buttons untouched means the robot is holding both balls)

        // If there are any number of balls in the robot, in any conveyance stage
        // set isRobotEmpty to false
        if(Robot.CONVEYANCE_SUBSYSTEM.getConveyanceOneSubsystemHasBall() == true || Robot.FEEDER_SUBSYSTEM.getFeederSubsystemHasBall() == true || isBallBetweenSensors == true) {
            isRobotEmpty = false;
        }
        // If there is a ball in conveyance stage 1, no ball in conveyance stage 2 (feeder subsystem), and the robot is not empty
        // set isBallBetweenSensors to true
        if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceOneSubsystemHasBall() == true && Robot.FEEDER_SUBSYSTEM.getFeederSubsystemHasBall() == false && isRobotEmpty == false) {
            isBallBetweenSensors = true;
            isRobotEmpty = false;
            Robot.FEEDER_SUBSYSTEM.setConveyanceNormalSpeedForward();
            System.out.println("Conveyance motor running; one ball in conveyance 1");
        }
        // If the ball state is in between sensors on the conveyance,
        // continue to run conveyance stage 2
        if(isBallBetweenSensors == true) {
            isRobotEmpty = false;
            Robot.FEEDER_SUBSYSTEM.setConveyanceNormalSpeedForward();
            System.out.println("Conveyance motor running; ball in between sensors");
        }
        // If there is a ball completely within conveyance stage 2,
        // set isBallBetweenSensors to false and stop conveyance stage 2
        if(Robot.FEEDER_SUBSYSTEM.getFeederSubsystemHasBall() == true) {
            isBallBetweenSensors = false;
            isRobotEmpty = false;
            Robot.FEEDER_SUBSYSTEM.stopConveyance();
            System.out.println("Conveyance motor is not running; ball in conveyance stage 2 (robot full)");
        }
        // If there are no balls in the robot,
        // stop conveyance stage 2
        if(Robot.CONVEYANCE_SUBSYSTEM.getConveyanceOneSubsystemHasBall() == false && Robot.FEEDER_SUBSYSTEM.getFeederSubsystemHasBall() == false && isBallBetweenSensors == false) {
            isRobotEmpty = true;
        }
        if(isRobotEmpty = true) {
            Robot.FEEDER_SUBSYSTEM.stopConveyance();
            System.out.println("Conveyance motor is not running; robot is empty");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.FEEDER_SUBSYSTEM.stopConveyance();
        isBallBetweenSensors = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}