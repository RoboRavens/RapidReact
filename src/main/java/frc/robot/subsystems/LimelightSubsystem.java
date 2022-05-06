package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry _tx = table.getEntry("tx");
  private NetworkTableEntry _ty = table.getEntry("ty");
  private NetworkTableEntry _ta = table.getEntry("ta"); 
  private NetworkTableEntry _tv = table.getEntry("tv"); 
 

  NetworkTableEntry ledMode = table.getEntry("ledMode");

  public int _ledState = 3;
  public int camMode = 0;
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance", getDistance());
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

  public double getDistance() {
  double targetOffsetAngle_Vertical = _ty.getDouble(0.0);

  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 56;

  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 36;

  // distance from the target to the floor
  double goalHeightInches = 104;

  double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  //calculate distance
  double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
  return distanceFromLimelightToGoalInches;
  }

  /**
   * Gets the raw limelight offset.
   * @return the raw limelight offset.
   */
  public double getRawTargetOffsetAngle() {
    return _tx.getDouble(0.0);
  }

  public double getRawYOffset() {
    return _ty.getDouble(0.0);
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
 