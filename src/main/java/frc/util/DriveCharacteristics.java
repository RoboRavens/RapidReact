package frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DriveCharacteristics {
    private double _previousTime = Timer.getFPGATimestamp();

    private double _previousPosition = 0;
    private double _previousVelocity = 0;
    private double _previousVelocityMax = 0;
    private double _previousVelocityAccelerationMax = 0;

    private double _previousAngle = 0;
    private double _previousAngularVelocity = 0;
    private double _previousAngularVelocityMax = 0;
    private double _previousAngularAccelerationMax = 0;

    private boolean reset = false;
    
    private final NetworkTableEntry _nteTime;
    private final NetworkTableEntry _nteTimeDelta;

    private final NetworkTableEntry _ntePosition;
    private final NetworkTableEntry _nteVelocity;
    private final NetworkTableEntry _nteVelocityMax;
    private final NetworkTableEntry _nteVelocityAcceleration;
    private final NetworkTableEntry _nteVelocityAccelerationMax;

    private final NetworkTableEntry _nteAngle;
    private final NetworkTableEntry _nteAngularVelocity;
    private final NetworkTableEntry _nteAngularVelocityMax;
    private final NetworkTableEntry _nteAngularAcceleration;
    private final NetworkTableEntry _nteAngularAccelerationMax;

    public DriveCharacteristics() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drive Characteristics");
        _nteTime = tab.add("Time", 0.0).withPosition(2, 0).withSize(1, 1).getEntry();
        _nteTimeDelta = tab.add("Time Delta", 0.0).withPosition(2, 1).withSize(1, 1).getEntry();

        _ntePosition = tab.add("Position", 0.0).withPosition(3, 0).withSize(1, 1).getEntry();
        _nteVelocity = tab.add("Velocity", 0.0).withPosition(3, 1).withSize(1, 1).getEntry();
        _nteVelocityMax = tab.add("Velocity Max", 0.0).withPosition(3, 2).withSize(1, 1).getEntry();
        _nteVelocityAcceleration = tab.add("Velocity Acc", 0.0).withPosition(3, 3).withSize(1, 1).getEntry();
        _nteVelocityAccelerationMax = tab.add("Velocity Acc Max", 0.0).withPosition(3, 4).withSize(1, 1).getEntry();

        _nteAngle = tab.add("Angle", 0.0).withPosition(4, 0).withSize(1, 1).getEntry();
        _nteAngularVelocity = tab.add("Angular Vel", 0.0).withPosition(4, 1).withSize(1, 1).getEntry();
        _nteAngularVelocityMax = tab.add("Angular Vel Max", 0.0).withPosition(4, 2).withSize(1, 1).getEntry();
        _nteAngularAcceleration = tab.add("Angular Acc", 0.0).withPosition(4, 3).withSize(1, 1).getEntry();
        _nteAngularAccelerationMax = tab.add("Angular Acc Max", 0.0).withPosition(4, 4).withSize(1, 1).getEntry();
    }

    public void reset() {
        reset = true;
    }

    public void update(Pose2d pose, double angleInDegrees) {
        if (reset) {
            _previousPosition = pose.getX();
            _previousVelocity = 0;
            _previousVelocityMax = 0;
            _previousVelocityAccelerationMax = 0;
        
            _previousAngle = Math.toRadians(angleInDegrees);
            _previousAngularVelocity = 0;
            _previousAngularVelocityMax = 0;
            _previousAngularAccelerationMax = 0;

            reset = false;
            return;
        }

        var time = Timer.getFPGATimestamp();
        var timeDelta = time - _previousTime;
        _nteTime.setDouble(time);
        _nteTimeDelta.setDouble(timeDelta);
        _previousTime = time;

        var position = pose.getX();
        var velocity = (position - _previousPosition) / timeDelta;
        var velocityMax = Math.max(velocity, _previousVelocityMax);
        var velocityAcceleration = (velocity - _previousVelocity) / timeDelta;
        var velocityAccelerationMax = Math.max(velocityAcceleration, _previousVelocityAccelerationMax);
        _ntePosition.setDouble(position);
        _nteVelocity.setDouble(velocity);
        _nteVelocityMax.setDouble(velocityMax);
        _nteVelocityAcceleration.setDouble(velocityAcceleration);
        _nteVelocityAccelerationMax.setDouble(velocityAccelerationMax);
        _previousPosition = position;
        _previousVelocity = velocity;
        _previousVelocityMax = velocityMax;
        _previousVelocityAccelerationMax = velocityAccelerationMax;

        var angle = Math.toRadians(angleInDegrees);
        var angularVelocity = (angle - _previousAngle) / timeDelta;
        var angularVelocityMax = Math.max(angularVelocity, _previousAngularVelocityMax);
        var angularAcceleration = (angularVelocity - _previousAngularVelocity) / timeDelta;
        var angularAccelerationMax = Math.max(angularAcceleration, _previousAngularAccelerationMax);
        _nteAngle.setDouble(angle);
        _nteAngularVelocity.setDouble(angularVelocity);
        _nteAngularVelocityMax.setDouble(angularVelocityMax);
        _nteAngularAcceleration.setDouble(angularAcceleration);
        _nteAngularAccelerationMax.setDouble(angularAccelerationMax);
        _previousAngle = angle;
        _previousAngularVelocity = angularVelocity;
        _previousAngularVelocityMax = angularVelocityMax;
        _previousAngularAccelerationMax = angularAccelerationMax;
    }
}
