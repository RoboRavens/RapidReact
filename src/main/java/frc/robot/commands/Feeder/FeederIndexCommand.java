package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FeederIndexCommand extends CommandBase {

    private boolean conveyanceOneHadBall;
    boolean conveyanceBallIsCorrect = false; // Should be replaced by a method call to get the sensor input
    boolean feederBallIsCorrect = false; // Should be replaced by a method call to get the sensor input
    
    public FeederIndexCommand() {
        addRequirements(Robot.FEEDER_SUBSYSTEM);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Robot.FEEDER_SUBSYSTEM.getFeederHasBall() && feederBallIsCorrect) {
            Robot.FEEDER_SUBSYSTEM.conveyanceStop();
            conveyanceOneHadBall = false;
        }
        else if (Robot.FEEDER_SUBSYSTEM.getFeederHasBall() && feederBallIsCorrect == false) {
            Robot.SHOOTER_SUBSYSTEM.shootWrongColorBall();
            conveyanceOneHadBall = false;
        }
        else if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() || conveyanceOneHadBall) {
            Robot.FEEDER_SUBSYSTEM.setConveyanceNormalSpeedForward();
            conveyanceOneHadBall = true;
        }
        else {
            Robot.SHOOTER_SUBSYSTEM.stopMotor();
            Robot.FEEDER_SUBSYSTEM.conveyanceStop();
            Robot.FEEDER_SUBSYSTEM.feederWheelStop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.FEEDER_SUBSYSTEM.conveyanceStop();
        conveyanceOneHadBall = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}