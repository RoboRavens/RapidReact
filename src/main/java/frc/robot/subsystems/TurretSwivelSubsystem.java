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
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.util.TurretCalibration;

public class TurretSwivelSubsystem extends SubsystemBase {

    private TalonSRX _turretMotor;
    private TurretCalibration _shot;

    public TurretSwivelSubsystem() {
        _turretMotor = new TalonSRX(RobotMap.TURRET_MOTOR);

        _turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        setShot(Constants.TURRET_DEFAULT_PID);

        setEncoder(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Angle", getAngle());
        SmartDashboard.putNumber("Turret RAW Encoder", _turretMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Turret Target", _shot.target);
        SmartDashboard.putBoolean("Turret Target Sighted", Robot.LIMELIGHT_SUBSYSTEM.hasTargetSighted());
    }

    @Override
    public void simulationPeriodic() {
        
    }

    public void defaultCommand() {
        
    }

    public double getAngle() {
        return _turretMotor.getSelectedSensorPosition() / Constants.TURRET_ENCODER_RATIO;
    }

    /**
     * Aims to the input angle. Will stop at edges of deadzone and flip if target is past deadzone.
     * @param angle - the angle in degrees.
     */
    public void goToAngle(double angle) {
        if(Math.abs(angle) > (360 - Constants.TURRET_RANGE)) { //If angle is overshooting bounds farther than the deadzone...
            angle += (Math.abs(angle) / angle) * -360; //Flips angle; adds 360 with an inverted sign to whatever angle is (if angle is +, add - and vice versa)
        } else { //If angle is over bounds but IN deadzone...
            angle = Math.max(angle, -1 * Constants.TURRET_RANGE); //Limit to turret range pos/neg
            angle = Math.min(angle, Constants.TURRET_RANGE);
        }
        _turretMotor.set(ControlMode.Position, angle * Constants.TURRET_ENCODER_RATIO);
        _shot.target = angle;
    }

    /**
     * Sets the subsystem's _shot value, along with all PID configs for the turret motor
     * @param shot - The TurretCalibration value to set _shot to
     */
    public void setShot(TurretCalibration shot) {
        _turretMotor.config_kF(Constants.TURRET_IDX, shot.kF, Constants.TURRET_TIMEOUT_MS);
        _turretMotor.config_kP(Constants.TURRET_IDX, shot.kP, Constants.TURRET_TIMEOUT_MS);
        _turretMotor.config_kI(Constants.TURRET_IDX, shot.kI, Constants.TURRET_TIMEOUT_MS);
        _turretMotor.config_kD(Constants.TURRET_IDX, shot.kD, Constants.TURRET_TIMEOUT_MS);

        _shot = shot;
    }

    public TurretCalibration getShot() {
        return _shot;
    }

    private void setEncoder(double sensorPos) {
        _turretMotor.setSelectedSensorPosition(sensorPos);
    }

    public boolean getIsAtTarget() {
        return (getAngle() > _shot.target - Constants.TURRET_AIM_ALLOWANCE && getAngle() < _shot.target + Constants.TURRET_AIM_ALLOWANCE);
    }

    public void shootWrongColorBall() {
        
    }
}