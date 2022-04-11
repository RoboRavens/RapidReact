package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonFX _climberMotor;
	private DoubleSolenoid _climberBrake;

	private double _extendedTarget = Constants.CLIMBER_EXTEND_ENCODER_TARGET;
	private double _retractedTarget = 0;
	private double _encoderAccuracyRange = Constants.CLIMBER_ENCODER_ACCURACY_RANGE;
	private boolean _override = false;
	private boolean _isClimbing;

    public ClimberSubsystem() {
		_climberMotor = new WPI_TalonFX(RobotMap.LEFT_CLIMBER_MOTOR);
		_climberMotor.configFactoryDefault();
		_climberMotor.setNeutralMode(NeutralMode.Brake);
		_climberMotor.setSelectedSensorPosition(0);

    _climberBrake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMBER_EXTENSION_SOLENOID, RobotMap.CLIMBER_RETRACTION_SOLENOID);
	}

	public void turnOverrideOn() {
		this._override = true;
	}

	public void turnOverrideOff() {
		this._override = false;
	}

	public void brakeClimber() {
		_climberBrake.set(Value.kForward);

		_isClimbing = false;
	}

	private void releaseClimberBrake() {
		_climberBrake.set(Value.kReverse);
		
		_isClimbing = true;
	}

	public boolean getIsClimbing() {
		return _isClimbing;
	}

	public void extendSlowly() {
		this.extend(Constants.CLIMBER_EXTEND_SOWLY_POWER_MAGNITUDE);
	}

	public void extend() {
		this.extend(Constants.CLIMBER_EXTEND_SLOW_POWER_MAGNITUDE);
	}

	public void extend(double power) {
		if (isAtEncoderExtensionLimit() == false || _override == true) {
			this.releaseClimberBrake();
			// _climberMotor.set(ControlMode.PercentOutput, power);
			setVoltage(power);
		}
	}

	public void retractSlowly() {
		this.retract(Constants.CLIMBER_RETRACT_SOWLY_POWER_MAGNITUDE);
	}

	public void retract() {
		this.retract(Constants.CLIMBER_RETRACT_POWER_MAGNITUDE);
	}

	public void retract(double power) {
		if (isAtEncoderRetractionLimit() == false || _override == true) {
			this.releaseClimberBrake();
			// _climberMotor.set(ControlMode.PercentOutput, power);
			setVoltage(power);
		}
	}

	// The better way to do this would be to update the constant values on
  // the methods that call this, but until we know it works I don't want
  // to change all the code to do that so we'll just do the conversion here.
	public void setVoltage(double percentOutput) {
		double voltage = percentOutput * 12.0;

		_climberMotor.setVoltage(voltage);
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

	public void periodic() {
		// SmartDashboard.putNumber("Climber Encoder", _climberMotor.getSelectedSensorPosition());
		// SmartDashboard.putBoolean("Climber Override", _override);
	}

	public void stop() {
		// _climberMotor.set(ControlMode.PercentOutput, 0);
		setVoltage(0);
	}

	public void holdPosition() {
		brakeClimber();
		// _climberMotor.set(ControlMode.PercentOutput, Constants.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
		setVoltage(Constants.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
	}

	public void defaultCommand() {
		this.holdPosition();
	}

	public boolean climberIsExtended() {
		boolean climberIsExtended = false;

		if (_climberMotor.getSelectedSensorPosition() >= Constants.CLIMBER_IS_EXTENDED_ENCODER_THRESHOLD) {
			climberIsExtended = true;
		}
		
		return climberIsExtended;
	}

	public double getEncoderPosition() {
		return _climberMotor.getSelectedSensorPosition();
	}
}
