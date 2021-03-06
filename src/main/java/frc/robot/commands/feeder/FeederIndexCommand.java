package frc.robot.commands.feeder;

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
        else if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall() || conveyanceOneHadBall) {
            Robot.FEEDER_SUBSYSTEM.setConveyanceNormalSpeedForward();
            conveyanceOneHadBall = true;
        }
        else {
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