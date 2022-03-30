// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.util.ShooterCalibration;
import frc.util.ShooterCalibration;
import frc.util.ShooterCalibrationPair;
import frc.util.ShooterCalibrationPair;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX _backspinMotor;
    private TalonFX _topspinMotor;
    private ShooterCalibrationPair _shot;
    private int _shotTally = 0;
    private boolean _isShooting;
    private boolean _recovered;
    private boolean _autoShotSelect = true;
    private double _lastShotTime = 0;
    private double _arbitraryFeedForward = 0;

    int tarmacBackSpinRpm = 0;
    int tarmacBackSpinKp = 0;
    int tarmacBackSpinKi = 0;
    int tarmacBackSpinKd = 0;
    int tarmacBackSpinKf = 0;

    int tarmacTopSpinRpm = 0;
    int tarmacTopSpinKp = 0;
    int tarmacTopSpinKi = 0;
    int tarmacTopSpinKd = 0;
    int tarmacTopSpinKf = 0;

    public ShooterSubsystem() {
        _backspinMotor = new TalonFX(RobotMap.SHOOTER_BACKSPIN_MOTOR);
        _topspinMotor = new TalonFX(RobotMap.SHOOTER_TOPSPIN_MOTOR);
        _backspinMotor.setInverted(true);
        _topspinMotor.setInverted(false);
        _backspinMotor.setNeutralMode(NeutralMode.Coast);
        _topspinMotor.setNeutralMode(NeutralMode.Coast);
        this.setShot(Constants.TARMAC_SHOT_CALIBRATION_PAIR);
        _backspinMotor.setSelectedSensorPosition(0);
        _topspinMotor.setSelectedSensorPosition(0);

        //SmartDashboard.putNumber("Set", Constants.TARMAC_BACKSPIN_RPM);

        // Show the backspin and topspin PID values on smart dashboard
        SmartDashboard.putNumber("CustomShooter Target RPM", tarmacBackSpinRpm);
        SmartDashboard.putNumber("CustomShooter KP", tarmacBackSpinKp);
        SmartDashboard.putNumber("CustomShooter KI", tarmacBackSpinKi);
        SmartDashboard.putNumber("CustomShooter KD", tarmacBackSpinKd);
        SmartDashboard.putNumber("CustomShooter KF", tarmacBackSpinKd);

        SmartDashboard.putBoolean("update shooter pid button", false);
    }

    @Override
    public void periodic() {
        if (detectShot()) {
            _lastShotTime = Timer.getFPGATimestamp();
            _shotTally++;
        }

        updatePidValuesOnSmartDashboard();
        updateSmartDashboard();

        updateShotProfile();
        updateArbitraryFeedForward();

        readAndUpdateValueFromSmartDashboard();

    }

    public void readAndUpdateValueFromSmartDashboard() {
        // System.out.println("test");
        //var value = SmartDashboard.getNumber("Set", Constants.TARMAC_BACKSPIN_RPM);
        //SmartDashboard.putNumber("View", value);

        //Constants.TARMAC_BACKSPIN_RPM = (int)SmartDashboard.getNumber("Tarmac Test", Constants.TARMAC_BACKSPIN_RPM);
        // SmartDashboard.putNumber("Tarmac Test", Constants.TARMAC_BACKSPIN_RPM);

        boolean buttonBooleanValue = SmartDashboard.getBoolean("update shooter pid button", false);
        //SmartDashboard.putBoolean("button view", booleanValue);

        // Set the backspin PID values to the changed values on smart dashboard, divided by two because the values are for a single motor
        tarmacBackSpinRpm = (int) SmartDashboard.getNumber("CustomShooter Target RPM", tarmacBackSpinRpm);
        tarmacBackSpinKp = (int) SmartDashboard.getNumber("CustomShooter KP", tarmacBackSpinKp);
        tarmacBackSpinKi = (int) SmartDashboard.getNumber("CustomShooter KI", tarmacBackSpinKi);
        tarmacBackSpinKd = (int) SmartDashboard.getNumber("CustomShooter KD", tarmacBackSpinKd);
        tarmacBackSpinKf = (int) SmartDashboard.getNumber("CustomShooter KF", tarmacBackSpinKf);

        // Set the topspin PID values to the changed values on smart dashboard, divided by two because the values are for a single motor
        tarmacTopSpinRpm = (int) SmartDashboard.getNumber("CustomShooter Target RPM", tarmacTopSpinRpm);
        tarmacTopSpinKp = (int) SmartDashboard.getNumber("CustomShooter KP", tarmacTopSpinKp);
        tarmacTopSpinKi = (int) SmartDashboard.getNumber("CustomShooter KI", tarmacTopSpinKi);
        tarmacTopSpinKd = (int) SmartDashboard.getNumber("CustomShooter KD", tarmacTopSpinKd);
        tarmacTopSpinKf = (int) SmartDashboard.getNumber("CustomShooter KF", tarmacTopSpinKf);

        if(buttonBooleanValue) {
            ShooterCalibration newBackSpinShooterCalibration = new ShooterCalibration("updated backspin shooter calibration", 
                tarmacBackSpinRpm, 
                tarmacBackSpinKf, 
                tarmacBackSpinKp,
                tarmacBackSpinKi,
                tarmacBackSpinKd,
                Constants.TARMAC_BACKSPIN_VOLTAGE_CONTROL_SETPOINT);

            ShooterCalibration newTopSpinShooterCalibration = new ShooterCalibration("updated topspin shooter calibration", 
                tarmacTopSpinRpm, 
                tarmacTopSpinKf, 
                tarmacTopSpinKp,
                tarmacTopSpinKi,
                tarmacTopSpinKd,
                Constants.TARMAC_TOPSPIN_VOLTAGE_CONTROL_SETPOINT);
            
                ShooterCalibrationPair newShooterCalibrationPair = new ShooterCalibrationPair("updated shooter calibration pair", newBackSpinShooterCalibration, newTopSpinShooterCalibration);
            
            setShot(newShooterCalibrationPair);
            startMotor();

            SmartDashboard.putBoolean("update shooter pid button", false);
        }

        // When box is checked
        //  Create the shooter calibration pair
        //  Call setShot and startMotor
        //  Uncheck the box to show that the execution has finished
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Backspin Shooter Speed", getBackspinShooterRPM());
        SmartDashboard.putNumber("Topspin Shooter Speed", getTopspinShooterRPM());
        SmartDashboard.putString("Shooter PID", _shot._name);
        SmartDashboard.putNumber("Shot Count", getShotCount());
        SmartDashboard.putNumber("Last Shot Time", getLastShotTime());
        SmartDashboard.putNumber("Back Shooter Rot", _backspinMotor.getSelectedSensorPosition() / Constants.TALONFX_TICKS_PER_REVOLUTION);
        SmartDashboard.putNumber("Backspin Target Speed", _backspinMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Backspin Target RPM", _shot._backspinMotorCalibration.targetRPM);
        SmartDashboard.putNumber("Topspin Target RPM", _shot._topspinMotorCalibration.targetRPM);
        
        SmartDashboard.putNumber("Backspin AMPS", _backspinMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Topspin AMPS", _topspinMotor.getStatorCurrent());
    }

    public void updatePidValuesOnSmartDashboard() {

        // Set the backspin PID values to the changed values on smart dashboard, divided by two because the values are for a single motor
        Constants.TARMAC_BACKSPIN_RPM = (int) SmartDashboard.getNumber("Tarmac Target RPM", Constants.TARMAC_BACKSPIN_RPM);
        Constants.TARMAC_BACKSPIN_KP = (int) SmartDashboard.getNumber("Tarmac KP", Constants.TARMAC_BACKSPIN_KP);
        Constants.TARMAC_BACKSPIN_KI = (int) SmartDashboard.getNumber("Tarmac KI", Constants.TARMAC_BACKSPIN_KI);
        Constants.TARMAC_BACKSPIN_KD = (int) SmartDashboard.getNumber("Tarmac KD", Constants.TARMAC_BACKSPIN_KD);
        Constants.TARMAC_BACKSPIN_KF = (int) SmartDashboard.getNumber("Tarmac KF", Constants.TARMAC_BACKSPIN_KF);

        // Set the topspin PID values to the changed values on smart dashboard, divided by two because the values are for a single motor
        Constants.TARMAC_TOPSPIN_RPM = (int) SmartDashboard.getNumber("Tarmac Target RPM", Constants.TARMAC_BACKSPIN_RPM);
        Constants.TARMAC_TOPSPIN_KP = (int) SmartDashboard.getNumber("Tarmac KP", Constants.TARMAC_BACKSPIN_KP);
        Constants.TARMAC_TOPSPIN_KI = (int) SmartDashboard.getNumber("Tarmac KI", Constants.TARMAC_BACKSPIN_KI);
        Constants.TARMAC_TOPSPIN_KD = (int) SmartDashboard.getNumber("Tarmac KD", Constants.TARMAC_BACKSPIN_KD);
        Constants.TARMAC_TOPSPIN_KF = (int) SmartDashboard.getNumber("Tarmac KF", Constants.TARMAC_BACKSPIN_KF);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void defaultCommand() {
        stopMotor();
    }

    public double getBackspinShooterRPM() {
        return _backspinMotor.getSelectedSensorVelocity() * Constants.TALON_VELOCITY_TO_RPM * Constants.BACKSPIN_GEAR_RATIO;
    }

    public double getTopspinShooterRPM() {
        return _topspinMotor.getSelectedSensorVelocity() * Constants.TALON_VELOCITY_TO_RPM * Constants.TOPSPIN_GEAR_RATIO;
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
        return _shot;
    }

    public void updateArbitraryFeedForward() {
        double aff = 0;
        
        switch (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceState()) {
            case OFF:
                aff += 0;
                break;
            case EJECTING:
                aff += Constants.SHOOTER_CONVEYANCE_EJECTING_ARBITRARY_FEED_FORWARD;
                break;
            case INDEXING:
                aff += Constants.SHOOTER_CONVEYANCE_INDEXING_ARBITRARY_FEED_FORWARD;
                break;
            case INTAKING:
                aff += Constants.SHOOTER_CONVEYANCE_INTAKING_ARBITRARY_FEED_FORWARD;
                break;
        }

        _arbitraryFeedForward = aff;
    }

    /**
    * Starts the motor with the set shot type
    */
    public void startMotor() {
        _recovered = false;

        double backspinTargetMotorRPM =  (_shot._backspinMotorCalibration.targetRPM / Constants.BACKSPIN_GEAR_RATIO);
        double backspinTargetVelocity = backspinTargetMotorRPM * Constants.TALON_RPM_TO_VELOCITY;

        double topspinTargetMotorRPM =  (_shot._topspinMotorCalibration.targetRPM / Constants.TOPSPIN_GEAR_RATIO);
        double topspinTargetVelocity = topspinTargetMotorRPM * Constants.TALON_RPM_TO_VELOCITY;

        updateShooterTuningSmartDashboard(backspinTargetMotorRPM, topspinTargetMotorRPM, backspinTargetVelocity, topspinTargetVelocity);

        _backspinMotor.set(ControlMode.Velocity, backspinTargetVelocity);
        _topspinMotor.set(ControlMode.Velocity, topspinTargetVelocity);

        _backspinMotor.set(ControlMode.Velocity, backspinTargetVelocity, DemandType.ArbitraryFeedForward, _arbitraryFeedForward);
        _topspinMotor.set(ControlMode.Velocity, topspinTargetVelocity, DemandType.ArbitraryFeedForward, _arbitraryFeedForward);

        _isShooting = true;
    }

    private void updateShooterTuningSmartDashboard(double backspinTargetMotorRPM, double topspinTargetMotorRPM, double backspinTargetVelocity, double topspinTargetVelocity) {
        SmartDashboard.putNumber("Backspin Target MOTOR RPM", backspinTargetMotorRPM);
        SmartDashboard.putNumber("Topspin Target MOTOR RPM", topspinTargetMotorRPM);

        SmartDashboard.putNumber("Backspin Target VEL", backspinTargetVelocity);
        SmartDashboard.putNumber("Topspin Target VEL", topspinTargetVelocity);

        SmartDashboard.putNumber("Backspin CURRENT VEL", _backspinMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Topspin CURRENT VEL", _topspinMotor.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Backspin VEL ERROR", _backspinMotor.getClosedLoopError());
        SmartDashboard.putNumber("Topspin VEL ERROR", _topspinMotor.getClosedLoopError());

        SmartDashboard.putNumber("Backspin WHEEL targ RPM", backspinTargetMotorRPM * Constants.BACKSPIN_GEAR_RATIO);
        SmartDashboard.putNumber("Topspin WHEEL targ RPM", topspinTargetMotorRPM * Constants.TOPSPIN_GEAR_RATIO);

        SmartDashboard.putString("Conveyance State", Robot.CONVEYANCE_SUBSYSTEM.getConveyanceState().name());
        SmartDashboard.putNumber("AFF Value", _arbitraryFeedForward);
    }

    /**
    * Stops the motor (sets it to 0% power)
    */
    public void stopMotor() {
        _backspinMotor.set(ControlMode.Current, 0);
        _topspinMotor.set(ControlMode.Current, 0);
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

    public boolean motorsAreSpinning() {
        boolean motorsAreSpinning = false;

        if ((getBackspinShooterRPM() + getTopspinShooterRPM()) / 2 > 200) {
            motorsAreSpinning = true;
        }

        return motorsAreSpinning;
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


    public boolean getReadyToShootTarmac() {
        boolean readyToShoot = false;
        
        double rpm = (getBackspinShooterRPM() + getTopspinShooterRPM()) / 2;

        if (rpm > 2375) {
            readyToShoot = true;
        }

        return readyToShoot;
    }

    private void updateShotProfile() {
        if (_autoShotSelect) {
            setShotProfileAutomatically();
        }

    }

    private void setShotProfileAutomatically() {
        // The tarmac is always a good default shot.
        ShooterCalibrationPair shotToSet = Constants.TARMAC_SHOT_CALIBRATION_PAIR;
    
        if (Robot.LIMELIGHT_SUBSYSTEM.getRawYOffset() > Constants.MAX_LOW_GOAL_SHOT) { // Close to hub
          shotToSet = Constants.LOW_GOAL_SHOT_CALIBRATION_PAIR;
        }
        else if (Robot.LIMELIGHT_SUBSYSTEM.getRawYOffset() > Constants.MAX_TARMAC_SHOT) {
          shotToSet = Constants.TARMAC_SHOT_CALIBRATION_PAIR;
        }
        else if (Robot.LIMELIGHT_SUBSYSTEM.getRawYOffset() > Constants.MAX_AUTO_RADIUS_SHOT) {
          shotToSet = Constants.AUTO_RADIUS_SHOT_CALIBRATION_PAIR;
        }
        else if (Robot.LIMELIGHT_SUBSYSTEM.getRawYOffset() > Constants.MAX_LAUNCHPAD_SHOT) {
          shotToSet = Constants.LAUNCHPAD_SHOT_CALIBRATION_PAIR;
        }

        // If the feeder has a ball of the wrong color, we use a low goal profile for ejection.
        if (Robot.FEEDER_SUBSYSTEM.feederHasWrongColorCargo()) {
            shotToSet = Constants.LOW_GOAL_SHOT_CALIBRATION_PAIR;
        }

        // ONLY set the profile if it's not already set, to avoid excess CAN traffic.
        if (shotToSet._name.equals(Robot.SHOOTER_SUBSYSTEM.getShot()._name) == false) {
            Robot.SHOOTER_SUBSYSTEM.setShot(shotToSet);
        }
    }
      
    public void disableAutoShotSelect() {
        _autoShotSelect = false;
    }

    public void enableAutoShotSelect() {
        _autoShotSelect = true;
    }

    public boolean getAutoShotSelect() {
        return _autoShotSelect;
    }
}