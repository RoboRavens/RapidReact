// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.ShooterCalibration;

public class ShooterSubsystem extends SubsystemBase {

    //Replace the motor type ASAP
    private TalonFX shooterMotor1;
    private TalonFX shooterMotor2;
    private ShooterCalibration _shot;
    private boolean isShooting;

    public ShooterSubsystem() {
        shooterMotor1 = new TalonFX(RobotMap.SHOOTER_MOTOR_1);
        shooterMotor2 = new TalonFX(RobotMap.SHOOTER_MOTOR_2);
        shooterMotor2.follow(shooterMotor1);
        _shot = Constants.TARMAC_SHOT;
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
        this.stopMotor();
    }

    /**
     * Sets both shooter motors to the specified shot type
     * @param shot The ShooterCalibration shot to set the motors to
     */
    public void setShot(ShooterCalibration shot) {
        this.shooterMotor1.config_kF(Constants.SHOOTER_IDX, shot.kF, Constants.SHOOTER_TIMEOUT_MS);
        this.shooterMotor1.config_kP(Constants.SHOOTER_IDX, shot.kP, Constants.SHOOTER_TIMEOUT_MS);
        this.shooterMotor1.config_kI(Constants.SHOOTER_IDX, shot.kI, Constants.SHOOTER_TIMEOUT_MS);
        this.shooterMotor1.config_kD(Constants.SHOOTER_IDX, shot.kD, Constants.SHOOTER_TIMEOUT_MS);

        this.shooterMotor2.config_kF(Constants.SHOOTER_IDX, shot.kF, Constants.SHOOTER_TIMEOUT_MS);
        this.shooterMotor2.config_kP(Constants.SHOOTER_IDX, shot.kP, Constants.SHOOTER_TIMEOUT_MS);
        this.shooterMotor2.config_kI(Constants.SHOOTER_IDX, shot.kI, Constants.SHOOTER_TIMEOUT_MS);
        this.shooterMotor2.config_kD(Constants.SHOOTER_IDX, shot.kD, Constants.SHOOTER_TIMEOUT_MS);

        this._shot = shot;
    }

    /**
     * Returns shot name
     * @return String of the current shot name
     */
    public String getShot() {
        return this._shot.name;
    }

    /**
    * Starts the motor with the set shot type
    */
    public void startMotor() {
        this.shooterMotor1.set(ControlMode.Velocity, _shot.targetRPM);
        isShooting = true;
    }

    /**
    * Stops the motor (sets it to 0% power)
    */
    public void stopMotor() {
        this.shooterMotor1.set(ControlMode.PercentOutput, 0);
        isShooting = false;
    }

    public boolean getIsShooting() {
		if(isShooting) {
			return true;
		}
		else {
			return false;
		}
	}
}