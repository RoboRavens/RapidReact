package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry _tx = table.getEntry("tx");
  // private NetworkTableEntry _ty = table.getEntry("ty");
  private NetworkTableEntry _ta = table.getEntry("ta"); // area
  // private NetworkTableEntry _tl = table.getEntry("Tl"); // The pipeline’s latency contribution (ms) Add at least 11ms for image capture
  private NetworkTableEntry _tv = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
  // private NetworkTableEntry _ts = table.getEntry("ts"); // Skew or rotation (-90 degrees to 0 degrees)

  NetworkTableEntry ledMode = table.getEntry("ledMode");
  // double x = tx.getDouble(0.0);
  // double y = ty.getDouble(0.0);
  // double area = ta.getDouble(0.5);
  // double l = tl.getDouble(0.0); // The pipeline’s latency contribution (ms) Add at least 11ms for image capture
  // double v = tv.getDouble(0.0); // Whether the limelight has any valid targets (0 or 1)
  // double s = ts.getDouble(0.0); // Skew or rotation (-90 degrees to 0 degrees)
  public int _ledState = 3;
  public int camMode = 0; 

  public boolean hasTargetSighted() {
    return _tv.getDouble(0.0) == 1;
  }

  public double getTargetOffsetAngle() {
    return _tx.getDouble(0.0);
  }

  public double getArea(){
    return _ta.getDouble(0.0);
  }

  public void toggleLED() {
		if (_ledState == 0) {
			setBothLEDOn();
		} else if (_ledState == 3) {
			setOneLEDOn();
		}
	}

  public void setBothLEDOn() {
		_ledState = 3;
	}

	public void turnLEDOff() {
		ledMode.setNumber(1);
	}

	public void setOneLEDOn() {
		_ledState = 0;
	}

	public void turnLEDOn() {
		ledMode.setNumber(3);
	}
}
 