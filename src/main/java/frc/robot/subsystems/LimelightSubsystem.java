package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.util.ShooterCalibrationPair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry _tx = table.getEntry("tx");
  private NetworkTableEntry _ty = table.getEntry("ty");
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LIMELIGHT X OFFSET", _tx.getDouble(0.0));
    SmartDashboard.putNumber("LIMELIGHT Y OFFSET", _ty.getDouble(0.0));

    ShooterCalibrationPair shotToSet = Constants.LAUNCHPAD_SHOT_CALIBRATION_PAIR;

    if(_ty.getDouble(256) < Constants.LIMELIGHT_MAX_LOWER_HUB_DISTANCE) { // Close to hub
      shotToSet = Constants.LOW_GOAL_SHOT_CALIBRATION_PAIR;
    } else if(_ty.getDouble(256) < Constants.MAX_TARMAC_SHOT) {
      shotToSet = Constants.TARMAC_SHOT_CALIBRATION_PAIR;
    } else if(_ty.getDouble(256) < Constants.MAX_AUTO_RADIUS_SHOT) {
      shotToSet = Constants.AUTO_RADIUS_SHOT_CALIBRATION_PAIR;
    }

    if(shotToSet._name != Robot.SHOOTER_SUBSYSTEM.getShot()._name) {
      Robot.SHOOTER_SUBSYSTEM.setShot(shotToSet);
    }
  }

  public boolean isAligned() {
    boolean isAligned = false;

    if (hasTargetSighted() && Math.abs(this.getTargetOffsetAngle()) < Constants.LIMELIGHT_IS_ALIGNED_DEGREES) {
      isAligned = true;
    }

    return isAligned;
  }

  public boolean hasTargetSighted() {
    return _tv.getDouble(0.0) == 1;
  }

  /**
   * Gets the offset such that the ball when shot will hit the center of the target.
   * @return the offset such that the ball when shot will hit the center of the target.
   */
  public double getTargetOffsetAngle() {
    return this.getRawTargetOffsetAngle() + Constants.LIMELIGHT_CENTERED_OFFSET;
  }

  /**
   * Gets the raw limelight offset.
   * @return the raw limelight offset.
   */
  public double getRawTargetOffsetAngle() {
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
 