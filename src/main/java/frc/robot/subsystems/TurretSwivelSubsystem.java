// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.TurretCalibration;

public class TurretSwivelSubsystem extends SubsystemBase {

    private TalonSRX _turretMotor;
    private TurretCalibration _shot;

    public TurretSwivelSubsystem() {
        _turretMotor = new TalonSRX(RobotMap.TURRET_MOTOR);

        _turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        setShot(Constants.TURRET_DEFAULT_PID);
        resetEncoder();
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Turret Angle", getAngle());
        SmartDashboard.putNumber("Turret RAW Encoder", _turretMotor.getSelectedSensorPosition());
    }

    @Override
    public void simulationPeriodic() {
        
    }

    public void defaultCommand() {
        
    }

    public void holdTarget(double relativeChange) {
        double changedAngle = getAngle() + relativeChange;
        goToAngle(changedAngle);
    }

    public double getAngle() {
        return _turretMotor.getSelectedSensorPosition() * Constants.TURRET_ENCODER_RATIO;
    }

    public void flip() {
        goToAngle(-1 * getAngle());
    }

    public void seek() {
        
    }

    public void goToAngle(double angle) {
        SmartDashboard.putNumber("Turret Target", angle * Constants.TURRET_ENCODER_RATIO);
        _turretMotor.set(ControlMode.Position, angle * Constants.TURRET_ENCODER_RATIO);
    }

    public void setShot(TurretCalibration shot) {
        _turretMotor.config_kF(Constants.TURRET_IDX, shot.kF, Constants.TURRET_TIMEOUT_MS);
        _turretMotor.config_kP(Constants.TURRET_IDX, shot.kP, Constants.TURRET_TIMEOUT_MS);
        _turretMotor.config_kI(Constants.TURRET_IDX, shot.kI, Constants.TURRET_TIMEOUT_MS);
        _turretMotor.config_kD(Constants.TURRET_IDX, shot.kD, Constants.TURRET_TIMEOUT_MS);

        _shot = shot;
    }

    public String getShot() {
        return _shot.name;
    }

    public void resetEncoder() {
        setEncoder(0);
    }

    public void setEncoder(double sensorPos) {
        _turretMotor.setSelectedSensorPosition(sensorPos);
    }
}