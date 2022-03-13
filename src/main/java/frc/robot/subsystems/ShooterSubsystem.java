// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.ShooterCalibration;
import frc.util.ShooterCalibrationPair;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX _backspinMotor;
    private TalonFX _topspinMotor;
    private ShooterCalibrationPair _shot;
    private boolean _isShooting;
    private int _shotTally = 0;
    private boolean _recovered;
    private double _lastShotTime = 0;

    public ShooterSubsystem() {
        _backspinMotor = new TalonFX(RobotMap.SHOOTER_BACKSPIN_MOTOR);
        _topspinMotor = new TalonFX(RobotMap.SHOOTER_TOPSPIN_MOTOR);
        _backspinMotor.setInverted(true);
        _topspinMotor.setInverted(false);
        _backspinMotor.setNeutralMode(NeutralMode.Coast);
        _topspinMotor.setNeutralMode(NeutralMode.Coast);
        this.setShot(Constants.TARMAC_SHOT_CALIBRATION_PAIR);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(detectShot()) {
            _lastShotTime = Timer.getFPGATimestamp();
            _shotTally++;
        }

        SmartDashboard.putNumber("Backspin Shooter Speed", getBackspinShooterRPM());
        SmartDashboard.putNumber("Topspin Shooter Speed", getTopspinShooterRPM());
        SmartDashboard.putString("Shooter PID", getShot()._name);
        SmartDashboard.putNumber("Shot Count", getShotCount());
        SmartDashboard.putNumber("Last Shot Time", getLastShotTime());
        SmartDashboard.putNumber("Backspin Motor Current", _backspinMotor.getMotorOutputVoltage());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void defaultCommand() {
        stopMotor();
    }

    public double getBackspinShooterRPM() {
        return _backspinMotor.getSelectedSensorVelocity() / Constants.TALON_TPS_TO_RPM;
    }

    public double getTopspinShooterRPM() {
        return _topspinMotor.getSelectedSensorVelocity() / Constants.TALON_TPS_TO_RPM;
    }

    public void setShot(ShooterCalibrationPair shot) {
        setMotorShot(_backspinMotor, shot._backspinMotorCalibration);
        setMotorShot(_topspinMotor, shot._topspinMotorCalibration);

        this._shot = shot;
    }

    /**
     * Sets both shooter motors to the specified shot type
     * @param shot The ShooterCalibration shot to set the motors to
     */
    public void setMotorShot(TalonFX motor, ShooterCalibration shot) {
        motor.config_kF(Constants.SHOOTER_IDX, shot.kF, Constants.SHOOTER_TIMEOUT_MS);
        motor.config_kP(Constants.SHOOTER_IDX, shot.kP, Constants.SHOOTER_TIMEOUT_MS);
        motor.config_kI(Constants.SHOOTER_IDX, shot.kI, Constants.SHOOTER_TIMEOUT_MS);
        motor.config_kD(Constants.SHOOTER_IDX, shot.kD, Constants.SHOOTER_TIMEOUT_MS);
    }

    /**
     * Returns shot
     * @return Current ShooterCalibrationPair value
     */
    public ShooterCalibrationPair getShot() {
        return this._shot;
    }

    /**
    * Starts the motor with the set shot type
    */
    public void startMotor() {
        _recovered = false;
        _backspinMotor.set(ControlMode.Velocity, _shot._backspinMotorCalibration.targetRPM * Constants.TALON_TPS_TO_RPM);
        _topspinMotor.set(ControlMode.Velocity, _shot._topspinMotorCalibration.targetRPM * Constants.TALON_TPS_TO_RPM);
        _isShooting = true;
    }

    /**
    * Stops the motor (sets it to 0% power)
    */
    public void stopMotor() {
        _backspinMotor.set(ControlMode.Velocity, 0);
        _topspinMotor.set(ControlMode.Velocity, 0);
        _isShooting = false;
    }

    public void resetShotCount() {
        _shotTally = 0;
    }

    public int getShotCount() {
        return _shotTally;
    }

    public boolean motorsAreRecovered() {
        return backspinMotorIsRecovered() && topspinMotorIsRecovered();
    }

    public boolean backspinMotorIsRecovered() {
        boolean isAboveMinimumRPM = getBackspinShooterRPM() > _shot._backspinMotorCalibration.targetRPM - Constants.SHOOTER_TARGET_ALLOWANCE;
        boolean isBelowMaximumRPM = getBackspinShooterRPM() < _shot._backspinMotorCalibration.targetRPM + Constants.SHOOTER_TARGET_ALLOWANCE;

        return isAboveMinimumRPM && isBelowMaximumRPM;
    }

    public boolean topspinMotorIsRecovered() {
        boolean isAboveMinimumRPM = getTopspinShooterRPM() > _shot._topspinMotorCalibration.targetRPM - Constants.SHOOTER_TARGET_ALLOWANCE;
        boolean isBelowMaximumRPM = getTopspinShooterRPM() < _shot._topspinMotorCalibration.targetRPM + Constants.SHOOTER_TARGET_ALLOWANCE;

        return isAboveMinimumRPM && isBelowMaximumRPM;
    }

    /**
     * Tallies up a point if a ball is detected as being fired
     */
    public boolean detectShot() {
        // This variable can be used to turn off this function.
        boolean testingMode = false;

        if (testingMode == false) {
            return false;
        }

        boolean output = false;
        
        if (_backspinMotor.getClosedLoopTarget() == 0) { //If we should be stopped anyway then end because we don't want to misfire
            return false;
        }
        if (_recovered && motorsAreRecovered() == false) { //If we WERE be at the target yet now we aren't (Ball JUST fired)
            output = true;
        }

        _recovered = motorsAreRecovered();
        
        return output;
    }

    public double getLastShotTime() {
        return _lastShotTime;
    }

    public boolean getIsShooting() {
        return _isShooting;
    }

}