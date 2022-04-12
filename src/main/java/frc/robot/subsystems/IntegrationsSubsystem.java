package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntegrationsSubsystem extends SubsystemBase {
    private boolean motorsAreSpinning = false;
    private Timer _timer = new Timer();
    
    public IntegrationsSubsystem() {

    }

    public void updateShooterMotorState() {
        if (Robot.SHOOTER_SUBSYSTEM.motorsAreSpinning()) {
            motorsAreSpinning = true;
        }
        else {
            motorsAreSpinning = false;
        }
    }

    public boolean isRobotFinishedShooting() {
        if (motorsAreSpinning) {
            if (Robot.getRobotCargoInventory() == 0) {
                _timer.start();
            } else {
                _timer.reset();
            }
        }
            
        if (_timer.get() > .3) {
            return true;
        }

        return false;
    }

    @Override
    public void periodic() {
        updateShooterMotorState();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    
}
