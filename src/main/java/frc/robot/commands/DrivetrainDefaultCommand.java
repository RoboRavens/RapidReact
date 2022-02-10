package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.controls.AxisCode;
import frc.controls.Gamepad;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DrivetrainDefaultCommand extends CommandBase {
    private final DriveTrainSubsystem _drivetrainSubsystem;
    private final Gamepad _gamepad;

    public DrivetrainDefaultCommand(DriveTrainSubsystem drivetrainSubsystem, Gamepad gamepad) {
        _drivetrainSubsystem = drivetrainSubsystem;
        _gamepad = gamepad;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        _drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        _gamepad.getAxis(AxisCode.LEFTSTICKX), // x translation
                        _gamepad.getAxis(AxisCode.LEFTSTICKY), // y translation
                        _gamepad.getAxis(AxisCode.RIGHTSTICKX) * 5, // rotation
                        _drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}