// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.TurretCalibration;

public class TurretSwivelSubsystem extends SubsystemBase {

    private TalonSRX _turretMotor;
    private TurretCalibration _shot;

    public TurretSwivelSubsystem() {
        _turretMotor = new TalonSRX(RobotMap.TURRET_MOTOR);
        setShot(Constants.TURRET_DEFAULT_PID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
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

    }

    public void seek() {
        
    }

    public void goToAngle(double angle) {
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
}