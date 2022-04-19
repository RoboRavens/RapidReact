package frc.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class Gamepad {
	private Joystick _joystick;

	public Gamepad(Joystick joystick) {
		_joystick = joystick;
	}

	public Gamepad(int port) {
		_joystick = new Joystick(port);
	}

	public boolean getButtonValue(ButtonCode button) {
		return _joystick.getRawButton(button.value);
	}

	public JoystickButton getButton(ButtonCode button) {
		return new JoystickButton(_joystick, button.value);
	}

	public boolean getAxisIsPressed(AxisCode axis) {
		boolean isPressed = false;
		double axisValue = this.getAxis(axis);

		if (axisValue >= Constants.AXIS_IS_PRESSED_VALUE) {
			isPressed = true;
		}

		return isPressed;
	}

	public double getAxis(AxisCode axis) {
		return _joystick.getRawAxis(axis.value);
	}

	public void setRumbleOn() {
        _joystick.setRumble(RumbleType.kLeftRumble, 1);
        _joystick.setRumble(RumbleType.kRightRumble, 1);
	}

	public void setRumbleOff() {
		_joystick.setRumble(RumbleType.kLeftRumble, 0);
        _joystick.setRumble(RumbleType.kRightRumble, 0);
	}
}