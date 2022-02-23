package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.ClimberCalibrations;

public class ClimberSubsystem extends SubsystemBase {

    private TalonFX _leftClimberMotor;
	private TalonFX _rightClimberMotor;

	private boolean targetIsExtended = false;

	private ClimberCalibrations _shot;

	private int extendedTarget = 0;
	private int retractedTarget = 0;

	private int leftSideEncoderTarget = 60000;
	private int rightSideEncoderTarget = 63000;

	private double extendMagnitude = 1; // Constants file is 1.0
	private double retractMagnitude = -.5; // Constants file is -.4

	// Set excessively generously for testing
	//private int encoderAccuracyRange = 15000;

	// A more reasonable value
	private int encoderAccuracyRange = 2000;

	private int defaultEncoderAccuracyRange = encoderAccuracyRange;

    public ClimberSubsystem() {

        //Will need to change motors to TalonFX for this season's robot
		_leftClimberMotor = new TalonFX(RobotMap.LEFT_CLIMBER_MOTOR);
		_rightClimberMotor = new TalonFX(RobotMap.RIGHT_CLIMBER_MOTOR);
		_leftClimberMotor.configFactoryDefault();
		_rightClimberMotor.configFactoryDefault();


		_leftClimberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		_rightClimberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

		setTarget(Constants.CLIMBER_PID);

		// _leftClimberMotor.getSensorCollection().setQuadraturePosition(0, 10);
		// _rightClimberMotor.getSensorCollection().setQuadraturePosition(0, 10);


		// _shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, TalonSRXConstants.kPIDLoopIdx,
 	    // TalonSRXConstants.kTimeoutMs);

		// _climberMotor.configForwardLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX,
		// LimitSwitchNormal.NormallyOpen);
		// _climberMotor.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX,
		// LimitSwitchNormal.NormallyOpen);
		// _climberMotor2.configForwardLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX,
		// LimitSwitchNormal.NormallyOpen);
		// _climberMotor2.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX,
		// LimitSwitchNormal.NormallyOpen);

		_rightClimberMotor.setInverted(true);
	}

	public void setTarget(ClimberCalibrations shot) {
        _rightClimberMotor.config_kF(Constants.CLIMBER_IDX, shot.kF, Constants.CLIMBER_TIMEOUT_MS);
        _rightClimberMotor.config_kP(Constants.CLIMBER_IDX, shot.kP, Constants.CLIMBER_TIMEOUT_MS);
        _rightClimberMotor.config_kI(Constants.CLIMBER_IDX, shot.kI, Constants.CLIMBER_TIMEOUT_MS);
        _rightClimberMotor.config_kD(Constants.CLIMBER_IDX, shot.kD, Constants.CLIMBER_TIMEOUT_MS);

        _leftClimberMotor.config_kF(Constants.CLIMBER_IDX, shot.kF, Constants.CLIMBER_TIMEOUT_MS);
        _leftClimberMotor.config_kP(Constants.CLIMBER_IDX, shot.kP, Constants.CLIMBER_TIMEOUT_MS);
        _leftClimberMotor.config_kI(Constants.CLIMBER_IDX, shot.kI, Constants.CLIMBER_TIMEOUT_MS);
        _leftClimberMotor.config_kD(Constants.CLIMBER_IDX, shot.kD, Constants.CLIMBER_TIMEOUT_MS);

        _shot = shot;
    }

	public void setOverrideOn() {
		this.encoderAccuracyRange = 0;
	}

	public void setOverrideOff() {
		this.encoderAccuracyRange = this.defaultEncoderAccuracyRange;
	}

	private void extendLeftSide() {
		_leftClimberMotor.set(ControlMode.PercentOutput, extendMagnitude);
	}

	private void extendRightSide() {
		_rightClimberMotor.set(ControlMode.PercentOutput, extendMagnitude);
	}

	public void extendPidRightSide() {
		_rightClimberMotor.set(ControlMode.Position, _shot.target);
	}

	public void extendPidLeftSide() {
		_leftClimberMotor.set(ControlMode.Position, _shot.target);
	}

	private void retractLeftSide() {
		_leftClimberMotor.set(ControlMode.PercentOutput, retractMagnitude);
	}

	private void retractRightSide() {
		_rightClimberMotor.set(ControlMode.PercentOutput, retractMagnitude);
	}

	public void extend() {
		this.extendLeftSide();
		this.extendRightSide();
		/*
		if (Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_1)
				&& Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_2)) {
			this.set(Calibrations.CLIMBER_EXTEND_POWER_MAGNITUDE);
		}
		*/
	}

	public void retract() {
		this.retractLeftSide();
		this.retractRightSide();
		/*
		if (Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_1)
				&& Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_2)) {
			this.set(Calibrations.CLIMBER_RETRACT_POWER_MAGNITUDE);
		}
		*/
	}

	private void set(double magnitude) {
		_leftClimberMotor.set(ControlMode.PercentOutput, magnitude);
		_rightClimberMotor.set(ControlMode.PercentOutput, -1 * magnitude);
	}

	public boolean encodersShowExtended() {
		boolean bothSidesExtended = leftEncoderShowsExtended() && rightEncoderShowsExtended();
		
		return bothSidesExtended;
	}

	public boolean leftEncoderShowsExtended() {
		boolean extended = _leftClimberMotor.getSelectedSensorPosition() > (leftSideEncoderTarget - encoderAccuracyRange);
		return extended;
	}

	// The right encoder goes into negative values as the climber is extended.
	public boolean rightEncoderShowsExtended() {
	//	boolean extended = _rightClimberMotor.getSelectedSensorPosition() < (rightSideEncoderTarget + encoderAccuracyRange);
	boolean extended = _rightClimberMotor.getSelectedSensorPosition() > (rightSideEncoderTarget - encoderAccuracyRange);
		
		return extended;
	}



	public boolean encodersShowRetracted() {
		boolean bothSidesRetracted = leftEncoderShowsRetracted() && rightEncoderShowsRetracted();
		
		return bothSidesRetracted;
	}

	public boolean leftEncoderShowsRetracted() {
		boolean retracted = _leftClimberMotor.getSelectedSensorPosition() < (retractedTarget + encoderAccuracyRange);
		return retracted;
	}

	// The right encoder goes into negative values as the climber is extended.
	public boolean rightEncoderShowsRetracted() {
//		boolean retracted = _rightClimberMotor.getSelectedSensorPosition() > (retractedTarget - encoderAccuracyRange);
boolean retracted = _rightClimberMotor.getSelectedSensorPosition() < (retractedTarget + encoderAccuracyRange);

	return retracted;
	}

	public void setTargetExtended() {
		this.targetIsExtended = true;
	}

	public void setTargetRetracted() {
		this.targetIsExtended = false;

	}

	public void periodic() {
		// System.out.println("Climber motor 1 Vel:" + _climberMotor.getSensorCollection().getQuadraturePosition() + " Climber motor 2 Vel:" + _climberMotor2.getSensorCollection().getQuadratureVelocity());
		// System.out.println("Climber motor 1 Pos:" + _climberMotor.getSensorCollection().getQuadraturePosition() + " Climber motor 2 Pos:" + _climberMotor2.getSensorCollection().getQuadratureVelocity());
		

		// System.out.println("Climber motor 1 Vel:" + _leftClimberMotor.getSelectedSensorVelocity() + " Climber motor 2 Vel:" + _rightClimberMotor.getSelectedSensorVelocity());
		// System.out.println("Climber motor 1 Pos:" + _leftClimberMotor.getSelectedSensorPosition() + " Climber motor 2 Pos:" + _rightClimberMotor.getSelectedSensorPosition());
		

		// _climberMotor.getSensorCollection()
		
		
		// getIntegratedSensorPosition();

		// _shooterMotor.getSelectedSensorVelocity();
		// System.out.println(_climberMotor. );

		/*
		 * SmartDashboard.putBoolean("Left Climber Is At Extension Limit",
		 * this.leftMotorIsAtExtensionLimit());
		 * SmartDashboard.putBoolean("Left Climber Is At Retraction Limit",
		 * this.leftMotorIsAtRetractionLimit());
		 * SmartDashboard.putBoolean("Right Climber Is At Extension Limit",
		 * this.rightMotorIsAtExtensionLimit());
		 * SmartDashboard.putBoolean("Right Climber Is At Retraction Limit",
		 * this.rightMotorIsAtRetractionLimit());
		 */ }

	public void stop() {
		_leftClimberMotor.set(ControlMode.PercentOutput, 0);
		_rightClimberMotor.set(ControlMode.PercentOutput, 0);
	}

	// Not sure which side is which
	public void stopLeftSide() {
		_leftClimberMotor.set(ControlMode.PercentOutput, 0);
	}

	public void stopRightSide() {
		_rightClimberMotor.set(ControlMode.PercentOutput, 0);
	}

	// public boolean isAtExtensionLimitLimitSwitchVersion() {
	// 	return (leftMotorIsAtExtensionLimitLimitSwitchVersion() && rightMotorIsAtExtensionLimitLimitSwitchVersion());
	// }

	// public boolean isAtRetractionLimitLimitSwitchVersion() {
	// 	return (leftMotorIsAtRetractionLimitLimitSwitchVersion() && rightMotorIsAtRetractionLimitLimitSwitchVersion());
	// }

	// private boolean leftMotorIsAtExtensionLimitLimitSwitchVersion() {
	// 	return _leftClimberMotor.getSensorCollection().isRevLimitSwitchClosed();
	// }

	// private boolean rightMotorIsAtExtensionLimitLimitSwitchVersion() {
	// 	return _rightClimberMotor.getSensorCollection().isRevLimitSwitchClosed();
	// }

	// private boolean leftMotorIsAtRetractionLimitLimitSwitchVersion() {
	// 	return _leftClimberMotor.getSensorCollection().isFwdLimitSwitchClosed();
	// }

	// private boolean rightMotorIsAtRetractionLimitLimitSwitchVersion() {
	// 	return _rightClimberMotor.getSensorCollection().isFwdLimitSwitchClosed();
	// }

	public void holdPositionLeftSide() {
		_leftClimberMotor.set(ControlMode.PercentOutput, Constants.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
	}

	public void holdPositionRightSide() {
		_rightClimberMotor.set(ControlMode.PercentOutput, Constants.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
	}

	public void holdPosition() {
		_leftClimberMotor.set(ControlMode.PercentOutput, Constants.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
		_rightClimberMotor.set(ControlMode.PercentOutput, Constants.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
	}

	public void defaultCommand() {
		this.holdPosition();
	}

	public void setBasedOnTarget() {
        //had to cast this to an int due to an error
		int leftSidePosition = (int) _leftClimberMotor.getSelectedSensorPosition();
		int rightSidePosition = (int) _rightClimberMotor.getSelectedSensorPosition();

		if (targetIsExtended) {
			if (leftEncoderShowsExtended() == false) {
				extendLeftSide();
			}
			else {
				holdPositionLeftSide();
			}

			if (rightEncoderShowsExtended() == false) {
				extendRightSide();
			}
			else {
				holdPositionRightSide();
			}
		}
		// Target is retracted
		else {
			if (leftEncoderShowsRetracted() == false) {
				retractLeftSide();
			}
			else {
				stopLeftSide();
			}

			if (rightEncoderShowsRetracted() == false) {
				retractRightSide();
			}
			else {
				stopRightSide();
			}
		}
	}
}
