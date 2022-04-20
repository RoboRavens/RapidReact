package frc.controls;

public enum AxisCode {
	LEFTSTICKX(0),
	LEFTSTICKY(1),
	LEFTTRIGGER(2),
	RIGHTTRIGGER(3),
	RIGHTSTICKX(4),
	RIGHTSTICKY(5);

	public final int value;
	
	AxisCode(int initValue)
	{
		this.value = initValue;
	}
}
