package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.controls.AxisCode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.util.Deadband;

public class DrivetrainDefaultCommand extends CommandBase {
    private boolean _followLimelight = false;
    private PIDController _followLimelightPID = new PIDController(5, 1, 1);

    public DrivetrainDefaultCommand() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double x = Robot.GAMEPAD.getAxis(AxisCode.LEFTSTICKY) * -1 * DriveTrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND; // Robot.JOYSTICK.getRawAxis(1); // Positive x is away from your alliance wall.
        double y = Robot.GAMEPAD.getAxis(AxisCode.LEFTSTICKX) * -1 * DriveTrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND; // Robot.JOYSTICK.getRawAxis(0); // Positive y is to your left when standing behind the alliance wall.
        double r; // The angular rate of the robot.
        Rotation2d a = Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation(); // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.

        x = Deadband.adjustValueToZero(x, Constants.JOYSTICK_DEADBAND);
        y = Deadband.adjustValueToZero(y, Constants.JOYSTICK_DEADBAND);

        var limelightAngle = this.getLimelightTargetOffset();
        if (limelightAngle != null) {
            r = _followLimelightPID.calculate(limelightAngle.doubleValue());
            // SmartDashboard.putNumber("", r);
        } else {
            // _followLimelightPID.reset();
            r = Robot.GAMEPAD.getAxis(AxisCode.RIGHTSTICKX) * -1 * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND; // Robot.JOYSTICK.getRawAxis(2);
            r = Deadband.adjustValueToZero(r, Constants.JOYSTICK_DEADBAND);
            r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;
        }

        if (x == 0 && y == 0 && r == 0) {
            Robot.DRIVE_TRAIN_SUBSYSTEM.holdPosition();
        } else {
            Robot.DRIVE_TRAIN_SUBSYSTEM.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    x, // x translation
                    y, // y translation
                    r, // rotation
                    a // The angle of the robot as measured by a gyroscope.
                )
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.DRIVE_TRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public void followLimelight() {
        _followLimelight = true;
    }

    public void stopFollowingLimelight() {
        _followLimelight = false;
    }

    private Double getLimelightTargetOffset() {
        if (_followLimelight == false) {
            return null;
        }

        if (Robot.LIMELIGHT_SUBSYSTEM.hasTargetSighted()) {
            return Robot.LIMELIGHT_SUBSYSTEM.getTargetOffsetAngle();
        }
        
        return 0.0;
    }
}