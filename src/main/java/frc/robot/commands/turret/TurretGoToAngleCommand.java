package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TurretGoToAngleCommand extends CommandBase {
    private double _target = 0;

    public TurretGoToAngleCommand(double angle) {
        addRequirements(Robot.TURRET_SWIVEL_SUBSYSTEM);
        _target = angle;
    }

    @Override
    public void initialize() {
        Robot.TURRET_SWIVEL_SUBSYSTEM.setPidProfile(Constants.TURRET_DEFAULT_PID);
    }

    @Override
    public void execute() {
        Robot.TURRET_SWIVEL_SUBSYSTEM.goToAngle(_target);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
