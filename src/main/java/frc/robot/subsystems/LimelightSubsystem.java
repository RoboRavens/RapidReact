package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  
    static double Limit = 0.00;
	edu.wpi.first.networktables.NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tl = table.getEntry("Tl");
    NetworkTableEntry tv = table.getEntry("Tv");
    NetworkTableEntry ts = table.getEntry("ts");
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.5);
    double l = tl.getDouble(0.0); // The pipeline’s latency contribution (ms) Add at least 11ms for image capture
    double v = tv.getDouble(0.0); // Whether the limelight has any valid targets (0 or 1)
    double s = ts.getDouble(0.0); // Skew or rotation (-90 degrees to 0 degrees)
   
}


 