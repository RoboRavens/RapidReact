package frc.util;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class PathWeaver {
    public static Trajectory getTrajectoryFromFile(String filename) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + filename, ex.getStackTrace());
        }

        return null;
    }
}
