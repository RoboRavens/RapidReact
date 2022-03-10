// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.ShooterCalibration;

public class ShooterSubsystem extends SubsystemBase {

    //Replace the motor type ASAP
    private TalonSRX _shooterMotor1;
    private VictorSPX _shooterMotor2;
    private ShooterCalibration _shot;
    private boolean isShooting;
    private int _shotTally = 0;
    private boolean _recovered;
    private double _lastShotTime = 0;

    public ShooterSubsystem() {
        _shooterMotor1 = new TalonSRX(RobotMap.SHOOTER_MOTOR_1);
        _shooterMotor2 = new VictorSPX(RobotMap.SHOOTER_MOTOR_2);
        _shooterMotor2.follow(_shooterMotor1);
        _shooterMotor2.setInverted(true);
        _shot = Constants.TARMAC_SHOT;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(detectShot()) {
            _lastShotTime = Timer.getMatchTime();
            _shotTally++;
        }

        SmartDashboard.putNumber("Shooter Speed", getRPM());
        SmartDashboard.putString("Shooter PID", _shot.name);
        SmartDashboard.putNumber("Shot Count", getShotCount());
        SmartDashboard.putNumber("Last Shot Time", getLastShotTime());
        SmartDashboard.putNumber("Shooter Current", _shooterMotor1.getMotorOutputVoltage());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void defaultCommand() {
        stopMotor();
    }

    public double getRPM() {
        return _shooterMotor1.getSelectedSensorVelocity() / Constants.SHOOTER_VEL_TO_RPM;
    }

    /**
     * Sets both shooter motors to the specified shot type
     * @param shot The ShooterCalibration shot to set the motors to
     */
    public void setShot(ShooterCalibration shot) {
        _shooterMotor1.config_kF(Constants.SHOOTER_IDX, shot.kF, Constants.SHOOTER_TIMEOUT_MS);
        _shooterMotor1.config_kP(Constants.SHOOTER_IDX, shot.kP, Constants.SHOOTER_TIMEOUT_MS);
        _shooterMotor1.config_kI(Constants.SHOOTER_IDX, shot.kI, Constants.SHOOTER_TIMEOUT_MS);
        _shooterMotor1.config_kD(Constants.SHOOTER_IDX, shot.kD, Constants.SHOOTER_TIMEOUT_MS);

        _shooterMotor2.config_kF(Constants.SHOOTER_IDX, shot.kF, Constants.SHOOTER_TIMEOUT_MS);
        _shooterMotor2.config_kP(Constants.SHOOTER_IDX, shot.kP, Constants.SHOOTER_TIMEOUT_MS);
        _shooterMotor2.config_kI(Constants.SHOOTER_IDX, shot.kI, Constants.SHOOTER_TIMEOUT_MS);
        _shooterMotor2.config_kD(Constants.SHOOTER_IDX, shot.kD, Constants.SHOOTER_TIMEOUT_MS);

        _shot = shot;
    }

    /**
     * Returns shot name
     * @return String of the current shot name
     */
    public String getShot() {
        return _shot.name;
    }

    /**
    * Starts the motor with the set shot type
    */
    public void startMotor() {
        this.shooterMotor1.set(ControlMode.Velocity, _shot.targetRPM);
        _recovered = false;
        _shooterMotor1.set(ControlMode.Velocity, _shot.targetRPM * Constants.SHOOTER_VEL_TO_RPM);
        isShooting = true;
    }

    /**
    * Stops the motor (sets it to 0% power)
    */
    public void stopMotor() {
        this.shooterMotor1.set(ControlMode.PercentOutput, 0);
        _shooterMotor1.set(ControlMode.PercentOutput, 0);
        isShooting = false;
    }

    public void resetShotCount() {
        _shotTally = 0;
    }

    public int getShotCount() {
        return _shotTally;
    }

    public boolean isRecovered() {
        return getRPM() > _shot.targetRPM - Constants.SHOOTER_TARGET_ALLOWANCE && getRPM() < _shot.targetRPM + Constants.SHOOTER_TARGET_ALLOWANCE;
    }

    /**
     * Tallies up a point if a ball is detected as being fired
     */
    public boolean detectShot() {
        boolean output = false;
        if(_shooterMotor1.getClosedLoopTarget() == 0) { //If we should be stopped anyway then end because we don't want to misfire
            return false;
        }
        if(_recovered && !isRecovered()) { //If we WERE be at the target yet now we aren't (Ball JUST fired)
            output = true;
        }
        _recovered = isRecovered();
        return output;
    }

    public double getLastShotTime() {
        return _lastShotTime;
    }

    public boolean getIsShooting() {
        return isShooting;
    }

}