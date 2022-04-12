// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.util.TurretCalibration;

public class TurretSwivelSubsystem extends SubsystemBase {

    private WPI_TalonSRX _turretMotor;
    private TurretCalibration _shot;
    private BufferedDigitalInput _zeroLimit;
    private BufferedDigitalInput _clockwiseLimit; // NEGATIVE ANGLE
    private BufferedDigitalInput _counterClockwiseLimit; // POSITIVE ANGLE
    private boolean _enabled = true; // when false the turret will not move and the drivetrain will align shooter with the goal
    private double motorOutputCap = Constants.TURRET_MOTOR_TRACKING_OUTPUT_CAP;

    public TurretSwivelSubsystem() {
        _turretMotor = new WPI_TalonSRX(RobotMap.TURRET_MOTOR);
        _zeroLimit = new BufferedDigitalInput(RobotMap.TURRET_ZERO_LIMIT_DIO_CHANNEL, 1, false, false);
        _clockwiseLimit = new BufferedDigitalInput(RobotMap.TURRET_CLOCKWISE_LIMIT_DIO_CHANNEL, 1, false, false);
        _counterClockwiseLimit = new BufferedDigitalInput(RobotMap.TURRET_COUNTER_CLOCKWISE_LIMIT_DIO_CHANNEL, 1, false, false);
        
//        _turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        ErrorCode sensorError = _turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        // _turretMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        // _turretMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        this.setPidProfile(Constants.TURRET_DEFAULT_PID);
        this.setEncoder(0);
    }

    @Override
    public void periodic() {
        _zeroLimit.maintainState();
        _counterClockwiseLimit.maintainState();
        _clockwiseLimit.maintainState();

        SmartDashboard.putNumber("Turret Angle", getAngle());
        SmartDashboard.putNumber("Turret RAW Encoder", _turretMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Turret Target", _shot.target);
        SmartDashboard.putBoolean("Turret Target Sighted", Robot.LIMELIGHT_SUBSYSTEM.hasTargetSighted());

        SmartDashboard.putBoolean("Counterclock limit", _counterClockwiseLimit.get());
        SmartDashboard.putBoolean("Zero limit", _zeroLimit.get());
        SmartDashboard.putBoolean("Clockwise limit", _clockwiseLimit.get());

        this.checkAndSetHardwareLimit();
    }

    @Override
    public void simulationPeriodic() {
        
    }

    public void defaultCommand() {
                
    }

    public void setOutputCap(double cap) {
        // Only update the output cap if it's different than the current output cap.
        if (cap != motorOutputCap) {
            motorOutputCap = cap;
            _turretMotor.configClosedLoopPeakOutput(Constants.TURRET_IDX, cap);
        }
    }

    public double getAngle() {
        return _turretMotor.getSelectedSensorPosition() / Constants.ENCODER_TO_TURRET_RATIO;
    }

    /**
     * Aims to the input angle. Will stop at edges of deadzone and flip if target is past deadzone.
     * @param angle - the angle in degrees.
     * @apiNote Can handle -870 to 870.
     * @apiNote Must be called in a command's execute() method - the limit check must be called in repetition.
     */
    public void goToAngle(double angle) {
        if (Math.abs(angle) > 360 - Constants.TURRET_RANGE) { //If angle is overshooting bounds farther than the deadzone...
            angle += (Math.abs(angle) / angle) * -360; //Flips angle; adds 360 with an inverted sign to whatever angle is (if angle is +, add - and vice versa)
        }

        if (Math.abs(angle) - 180 > 0) {
            angle += (Math.abs(angle) / angle) * -360;
        }

        angle = Math.max(angle, -1 * Constants.TURRET_RANGE); //Limit to turret range pos/neg
        angle = Math.min(angle, Constants.TURRET_RANGE);
        if (_enabled) {
            _turretMotor.set(ControlMode.Position, angle * Constants.ENCODER_TO_TURRET_RATIO); //Mult by ratio
        } else {
            _turretMotor.set(ControlMode.PercentOutput, 0);
        }
        _shot.target = angle;
    }

    private void checkAndSetHardwareLimit() {
        if(_clockwiseLimit.get()) {
            this.setEncoder(Constants.TURRET_CLOCKWISE_HARDWARE_LIMIT * Constants.ENCODER_TO_TURRET_RATIO);
        }

        if(_counterClockwiseLimit.get()) {
            this.setEncoder(Constants.TURRET_COUNTER_CLOCKWISE_HARDWARE_LIMIT * Constants.ENCODER_TO_TURRET_RATIO);
        }

        // if (_zeroLimit.get()) {
        //     this.setEncoder(0);
        // }
    }

    /**
     * Sets the subsystem's _shot value, along with all PID configs for the turret motor
     * @param shot - The TurretCalibration value to set _shot to
     */
    public void setPidProfile(TurretCalibration shot) {
        _turretMotor.config_kF(Constants.TURRET_IDX, shot.kF, Constants.TURRET_TIMEOUT_MS);
        _turretMotor.config_kP(Constants.TURRET_IDX, shot.kP, Constants.TURRET_TIMEOUT_MS);
        _turretMotor.config_kI(Constants.TURRET_IDX, shot.kI, Constants.TURRET_TIMEOUT_MS);
        _turretMotor.config_kD(Constants.TURRET_IDX, shot.kD, Constants.TURRET_TIMEOUT_MS);

        _shot = shot;
    }

    public TurretCalibration getPidProfile() {
        return _shot;
    }

    private void setEncoder(double sensorPos) {
        _turretMotor.setSelectedSensorPosition(sensorPos);
    }

    public boolean getIsAtTarget() {
        return (getAngle() > _shot.target - Constants.TURRET_AIM_ALLOWANCE && getAngle() < _shot.target + Constants.TURRET_AIM_ALLOWANCE);
    }

    public void setTurretEnabled(boolean value) {
        _enabled = value;
    }

    public boolean getTurretEnabled() {
        return _enabled;
    }
}