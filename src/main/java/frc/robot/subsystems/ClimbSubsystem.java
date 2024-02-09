package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{
    
    private CANSparkMax climbMotorRight;
    private CANSparkMax climbMotorLeft;

    public ClimbSubsystem() {
        climbMotorRight = new CANSparkMax(ClimbConstants.kClimbMotorRightPort, MotorType.kBrushless);
        climbMotorLeft = new CANSparkMax(ClimbConstants.kClimbMotorLeftPort, MotorType.kBrushless);
    }

    public void setClimbSpeedPercentage(double climbSpeedPercentage) {
        climbMotorRight.set(climbSpeedPercentage);
        climbMotorLeft.set(climbSpeedPercentage);
    }
}    

