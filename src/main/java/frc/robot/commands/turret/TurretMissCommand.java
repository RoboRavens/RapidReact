package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TurretMissCommand extends CommandBase{

    private double _target;
    
    public TurretMissCommand() {
        addRequirements(Robot.TURRET_SWIVEL_SUBSYSTEM);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double angleOffset = Constants.TURRET_MISS_OFFSET;
        if(Robot.TURRET_SWIVEL_SUBSYSTEM.getAngle() >= 0) {
            angleOffset = -angleOffset;
        }
        
        _target = Robot.TURRET_SWIVEL_SUBSYSTEM.getAngle() + angleOffset;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.TURRET_SWIVEL_SUBSYSTEM.goToAngle(_target);
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