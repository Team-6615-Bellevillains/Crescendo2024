package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class FlipUtil {
    public static boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }
}
