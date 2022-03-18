package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ClimberSubsystem extends SubsystemBase {

    private TalonFX _climberMotor;

	private double _extendedTarget = Constants.CLIMBER_EXTEND_ENCODER_TARGET;
	private double _retractedTarget = 0;

	private double _extendMagnitude = 1; // Constants file is 1.0
	private double _retractMagnitude = -.5; // Constants file is -.4

	boolean _override = false;

	// Set excessively generously for testing
	//private int encoderAccuracyRange = 15000;

	// A more reasonable value
	private double _encoderAccuracyRange = Constants.CLIMBER_ENCODER_ACCURACY_RANGE;

	// private int defaultEncoderAccuracyRange = encoderAccuracyRange;

	// private Solenoid _climberBrakeRight;
	private Solenoid _climberBrakeLeftExtend;
	private Solenoid _climberBrakeLeftRetract;

	private boolean _isClimbing;

    public ClimberSubsystem() {
		_climberMotor = new TalonFX(RobotMap.LEFT_CLIMBER_MOTOR);
		_climberMotor.configFactoryDefault();
		_climberMotor.setNeutralMode(NeutralMode.Brake);		
		_climberMotor.setSelectedSensorPosition(0);

		// _climberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder)

    	_climberBrakeLeftExtend = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMBER_EXTENSION_SOLENOID);
		_climberBrakeLeftRetract = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMBER_RETRACTION_SOLENOID);
	}

	public void turnOverrideOn() {
		this._override = true;
	}

	public void turnOverrideOff() {
		this._override = false;
	}

	public void brakeClimber() {
		_climberBrakeLeftExtend.set(true);
		_climberBrakeLeftRetract.set(false);

		_isClimbing = false;
	}

	private void releaseClimberBrake() {
		_climberBrakeLeftExtend.set(false);
		_climberBrakeLeftRetract.set(true);
		
		_isClimbing = true;
	}

	public boolean getIsClimbing() {
		return _isClimbing;
	}

	public void extend() {
		// if (isAtEncoderExtensionLimit() == false || _override == true) {
			this.releaseClimberBrake();
			_climberMotor.set(ControlMode.PercentOutput, _extendMagnitude);
		//}
		
		/*
		if (Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_1)
				&& Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_2)) {
			this.set(Calibrations.CLIMBER_EXTEND_POWER_MAGNITUDE);
		}
		*/
	}

	public void retract() {
		//if (isAtEncoderRetractionLimit() == false || _override == true) {
			this.releaseClimberBrake();
			_climberMotor.set(ControlMode.PercentOutput, _retractMagnitude);
		//}
		
		
		/*
		if (Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_1)
				&& Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_2)) {
			this.set(Calibrations.CLIMBER_RETRACT_POWER_MAGNITUDE);
		}
		*/
	}

	public boolean isAtEncoderExtensionLimit() {
		boolean isAtExtensionLimit = false;
		
		double encoderPosition = _climberMotor.getSelectedSensorPosition();

		if (encoderPosition >= _extendedTarget - _encoderAccuracyRange) {
			isAtExtensionLimit = true;
		}

		return isAtExtensionLimit;
	}

	public boolean isAtEncoderRetractionLimit() {
		boolean isAtRetractionLimit = false;
		
		double encoderPosition = _climberMotor.getSelectedSensorPosition();

		if (encoderPosition <= _retractedTarget + _encoderAccuracyRange) {
			isAtRetractionLimit = true;
		}

		return isAtRetractionLimit;
	}

	public void periodic() {}

	public void stop() {
		_climberMotor.set(ControlMode.PercentOutput, 0);
	}

	public void holdPosition() {
		// The brake line should likely be uncommented but it's left commented for now
		// in case there's an error elsewhere in the code that would
		// result in a double-setting of the brake during operation.
		// brakeClimber();
		_climberMotor.set(ControlMode.PercentOutput, Constants.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
	}

	public void defaultCommand() {
		this.holdPosition();
	}
}
