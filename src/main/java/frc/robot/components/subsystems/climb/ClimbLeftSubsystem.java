package frc.robot.components.subsystems.climb;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbLeftSubsystem extends SubsystemBase {

    private final CANSparkMax climbMotorLeft;

    public ClimbLeftSubsystem() {
        climbMotorLeft = new CANSparkMax(ClimbConstants.kClimbMotorLeftPort, MotorType.kBrushless);
        climbMotorLeft.getEncoder().setPosition(0);
    }

    public void setClimbSpeedPercentage(double climbSpeedPercentage) {
        climbMotorLeft.set(climbSpeedPercentage);
    }

    public double getClimbRotations() {
        return climbMotorLeft.getEncoder().getPosition();
    }
}    

