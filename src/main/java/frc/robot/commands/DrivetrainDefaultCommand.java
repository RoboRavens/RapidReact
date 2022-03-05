package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.controls.AxisCode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.util.Deadband;

public class DriveTrainDefaultCommand extends CommandBase {
    private final DriveTrainSubsystem _drivetrainSubsystem;
    private final PIDController _driftCorrectionPID = new PIDController(Constants.DRIVE_ANGLE_DRIFT_CORRECTION_KP, Constants.DRIVE_ANGLE_DRIFT_CORRECTION_KI, Constants.DRIVE_ANGLE_DRIFT_CORRECTION_KD);

    private Rotation2d _desiredHeading = null;

    public DriveTrainDefaultCommand(DriveTrainSubsystem drivetrainSubsystem) {
        _drivetrainSubsystem = drivetrainSubsystem;
        _desiredHeading = _drivetrainSubsystem.getGyroscopeRotation();
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double x = Robot.GAMEPAD.getAxis(AxisCode.LEFTSTICKY) * -1; // Robot.JOYSTICK.getRawAxis(1); // Positive x is away from your alliance wall.
        double y = Robot.GAMEPAD.getAxis(AxisCode.LEFTSTICKX) * -1; // Robot.JOYSTICK.getRawAxis(0); // Positive y is to your left when standing behind the alliance wall.
        double r = Robot.GAMEPAD.getAxis(AxisCode.RIGHTSTICKX) * -1; // Robot.JOYSTICK.getRawAxis(2); // The angular rate of the robot.
        Rotation2d a = _drivetrainSubsystem.getGyroscopeRotation(); // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.

        x = Deadband.adjustValue(x, Constants.JOYSTICK_DEADBAND);
        y = Deadband.adjustValue(y, Constants.JOYSTICK_DEADBAND);
        r = Deadband.adjustValue(r, Constants.JOYSTICK_DEADBAND);

        r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;

        r = maintainGyroAngle(x,y,r);

        // System.out.println("gyro: " + a);
        //if (x == 0 && y == 0 && r == 0) {
        //    _drivetrainSubsystem.holdPosition();
        //} else {
            _drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    x, // x translation
                    y, // y translation
                    r, // rotation
                    a // The angle of the robot as measured by a gyroscope.
                )
            );
        //}
    }

    // if the user is not telling the robot to turn then maintain the angle from when the user first stopped moving
    private double maintainGyroAngle(double x, double y, double r) {
        // user is turning
        if (r != 0) {
            _desiredHeading = null;
            // SmartDashboard.putBoolean("Drift Correct Engadged", false);
            return r;
        }
        
        // the user stopped turning, and was turning previously
        if (_desiredHeading == null) {
            _desiredHeading = _drivetrainSubsystem.getGyroscopeRotation();
            _driftCorrectionPID.reset();
        }
        
        // only correct angle if moving, so the robot doesn't try to slowly turn while at a standstill
        if (x != 0 || y != 0) {
            r = _driftCorrectionPID.calculate(_drivetrainSubsystem.getGyroscopeRotation().getRadians(), _desiredHeading.getRadians());
        }
        
        // SmartDashboard.putNumber("Actual Heading", _drivetrainSubsystem.getGyroscopeRotation().getDegrees());
        // SmartDashboard.putNumber("Desired Heading", _desiredHeading.getDegrees());
        // SmartDashboard.putNumber("Drift Correct", r);
        // SmartDashboard.putBoolean("Drift Correct Engadged", r != 0);

        return r;
    }

    public void gyroReset() {
        _desiredHeading = null;
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}