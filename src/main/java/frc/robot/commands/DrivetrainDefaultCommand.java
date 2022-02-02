package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.controls.AxisCode;
import frc.controls.Gamepad;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.util.Deadband;

public class DrivetrainDefaultCommand extends CommandBase {
    private final DrivetrainSubsystem _drivetrainSubsystem;
    private final Gamepad _gamepad;

    public DrivetrainDefaultCommand(DrivetrainSubsystem drivetrainSubsystem, Gamepad gamepad) {
        _drivetrainSubsystem = drivetrainSubsystem;
        _gamepad = gamepad;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double x = _gamepad.getAxis(AxisCode.LEFTSTICKY); // Positive x is away from your alliance wall.
        double y = _gamepad.getAxis(AxisCode.LEFTSTICKX); // Positive y is to your left when standing behind the alliance wall.
        double r = _gamepad.getAxis(AxisCode.LEFTTRIGGER); // The angular rate of the robot.
        Rotation2d a = _drivetrainSubsystem.getGyroscopeRotation(); // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.

        x = Deadband.adjustValue(x, Constants.JOYSTICK_DEADBAND);
        y = Deadband.adjustValue(y, Constants.JOYSTICK_DEADBAND);
        r = Deadband.adjustValue(r, Constants.JOYSTICK_DEADBAND);

        r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;

        _drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x, // x translation
                        y, // y translation
                        r, // rotation
                        a
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}