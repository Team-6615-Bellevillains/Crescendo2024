package frc.robot.utils.enums;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlipUtil {
    public static boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        // SmartDashboard.putBoolean("Is Red", isRed);
        return isRed;
    }
}
