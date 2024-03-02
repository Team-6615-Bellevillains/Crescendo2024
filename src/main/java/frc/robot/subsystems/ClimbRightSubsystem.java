package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbRightSubsystem extends SubsystemBase  {
        
    private final CANSparkMax climbMotorRight;

    public ClimbRightSubsystem() {
        climbMotorRight = new CANSparkMax(ClimbConstants.kClimbMotorRightPort, MotorType.kBrushless);
    }

    public void setClimbSpeedPercentage(double climbSpeedPercentage) {
        climbMotorRight.set(climbSpeedPercentage);
    }
}    