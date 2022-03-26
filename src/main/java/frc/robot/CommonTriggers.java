package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommonTriggers {
    public static Trigger RobotHas2Balls = new Trigger(() -> {
        return Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall() && Robot.FEEDER_SUBSYSTEM.getFeederHasBall();
    });
}
