package frc.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriftCorrection {
    private static final PIDController _driftCorrectionPID = new PIDController(Constants.DRIVE_ANGLE_DRIFT_CORRECTION_KP, Constants.DRIVE_ANGLE_DRIFT_CORRECTION_KI, Constants.DRIVE_ANGLE_DRIFT_CORRECTION_KD);
    private static Rotation2d _desiredHeading = null;
    
    // if the user is not telling the robot to turn then maintain the angle from when the user first stopped moving
    public static double maintainGyroAngle(double x, double y, double r) {
        // user is turning
        if (r != 0) {
            _desiredHeading = null;
            // SmartDashboard.putBoolean("Drift Correct Engadged", false);
            return r;
        }
        
        // the user stopped turning, and was turning previously
        if (_desiredHeading == null) {
            _desiredHeading = Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation();
            _driftCorrectionPID.reset();
        }
        
        // only correct angle if moving, so the robot doesn't try to slowly turn while at a standstill
        if (x != 0 || y != 0) {
            r = _driftCorrectionPID.calculate(Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation().getRadians(), _desiredHeading.getRadians());
        }
        
        // SmartDashboard.putNumber("Actual Heading", _drivetrainSubsystem.getGyroscopeRotation().getDegrees());
        // SmartDashboard.putNumber("Desired Heading", _desiredHeading.getDegrees());
        // SmartDashboard.putNumber("Drift Correct", r);
        // SmartDashboard.putBoolean("Drift Correct Engadged", r != 0);

        return r;
    }

    public static void clearDesiredHeading() {
        _desiredHeading = null;
    }
}
