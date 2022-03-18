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
	private int retractedTarget = 0;

	private double extendMagnitude = 1; // Constants file is 1.0
	private double retractMagnitude = -.5; // Constants file is -.4

	// Set excessively generously for testing
	//private int encoderAccuracyRange = 15000;

	// A more reasonable value
	private int encoderAccuracyRange = 2000;

	private int defaultEncoderAccuracyRange = encoderAccuracyRange;

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

	public void setOverrideOn() {
		this.encoderAccuracyRange = 0;
	}

	public void setOverrideOff() {
		this.encoderAccuracyRange = this.defaultEncoderAccuracyRange;
	}

	private void extendLeftSide() {
		_climberMotor.set(ControlMode.PercentOutput, extendMagnitude);
	}

	private void retractLeftSide() {
		_climberMotor.set(ControlMode.PercentOutput, retractMagnitude);
	}

	public void extend() {
		this.releaseClimberBrake();
		this.extendLeftSide();
		/*
		if (Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_1)
				&& Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_2)) {
			this.set(Calibrations.CLIMBER_EXTEND_POWER_MAGNITUDE);
		}
		*/
	}

	public void retract() {
		this.releaseClimberBrake();
		this.retractLeftSide();
		
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

		if (encoderPosition >= _extendedTarget) {
			isAtExtensionLimit = true;
		}

		return isAtExtensionLimit;
	}

	public boolean encodersShowRetracted() {
		boolean bothSidesRetracted = leftEncoderShowsRetracted();
		
		return bothSidesRetracted;
	}

	public boolean leftEncoderShowsRetracted() {
		boolean retracted = _climberMotor.getSelectedSensorPosition() < (retractedTarget + encoderAccuracyRange);
		return retracted;
	}

	public void periodic() {}

	public void stop() {
		_climberMotor.set(ControlMode.PercentOutput, 0);
	}

	public void holdPosition() {
		_climberMotor.set(ControlMode.PercentOutput, Constants.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
	}

	public void defaultCommand() {
		this.holdPosition();
	}
}
