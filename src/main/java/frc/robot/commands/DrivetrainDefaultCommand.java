package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.controls.AxisCode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.util.Deadband;

public class DriveTrainDefaultCommand extends CommandBase {
    private final DriveTrainSubsystem _drivetrainSubsystem;

    public DriveTrainDefaultCommand(DriveTrainSubsystem drivetrainSubsystem) {
        _drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double x = Robot.GAMEPAD.getAxis(AxisCode.LEFTSTICKY); // Robot.JOYSTICK.getRawAxis(1); // Positive x is away from your alliance wall.
        double y = Robot.GAMEPAD.getAxis(AxisCode.LEFTSTICKX); // Robot.JOYSTICK.getRawAxis(0); // Positive y is to your left when standing behind the alliance wall.
        double r = Robot.GAMEPAD.getAxis(AxisCode.RIGHTSTICKX); // Robot.JOYSTICK.getRawAxis(2); // The angular rate of the robot.
        Rotation2d a = _drivetrainSubsystem.getGyroscopeRotation(); // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.

        x = Deadband.adjustValue(x, Constants.JOYSTICK_DEADBAND);
        y = Deadband.adjustValue(y, Constants.JOYSTICK_DEADBAND);
        r = Deadband.adjustValue(r, Constants.JOYSTICK_DEADBAND);

        r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;
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

    @Override
    public void end(boolean interrupted) {
        _drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}