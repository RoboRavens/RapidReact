package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TurretMissCommand extends CommandBase{
    public TurretMissCommand() {
        addRequirements(Robot.TURRET_SWIVEL_SUBSYSTEM);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angleOffset = (Robot.TURRET_SWIVEL_SUBSYSTEM.getAngle() >= 0 ? -Constants.TURRET_MISS_OFFSET : Constants.TURRET_MISS_OFFSET);
        Robot.TURRET_SWIVEL_SUBSYSTEM.goToAngle(Robot.TURRET_SWIVEL_SUBSYSTEM.getAngle() + angleOffset);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // new TurretSeekCommand();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Robot.TURRET_SWIVEL_SUBSYSTEM.getIsAtTarget();
    }
}