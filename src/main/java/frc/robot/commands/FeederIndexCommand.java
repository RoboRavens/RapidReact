package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FeederIndexCommand extends CommandBase {

    private boolean conveyanceOneHadBall;
    
    public FeederIndexCommand() {
        addRequirements(Robot.FEEDER_SUBSYSTEM);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Robot.FEEDER_SUBSYSTEM.getFeederHasBall()) {
            Robot.FEEDER_SUBSYSTEM.conveyanceStop();
            conveyanceOneHadBall = false;
        }
        else if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() || conveyanceOneHadBall) {
            Robot.FEEDER_SUBSYSTEM.setConveyanceNormalSpeedForward();
            conveyanceOneHadBall = true;
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