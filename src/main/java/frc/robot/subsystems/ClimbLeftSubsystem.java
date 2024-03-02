package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbLeftSubsystem extends SubsystemBase {

    private final CANSparkMax climbMotorLeft;

    public ClimbLeftSubsystem() {
        climbMotorLeft = new CANSparkMax(ClimbConstants.kClimbMotorLeftPort, MotorType.kBrushless);
    }

    public void setClimbSpeedPercentage(double climbSpeedPercentage) {
        climbMotorLeft.set(climbSpeedPercentage);
    }
}    

