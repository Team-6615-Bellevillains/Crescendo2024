package frc.robot.subsystems;

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Left Rotations", getClimbRotations());
    }

    public void setClimbSpeedPercentage(double climbSpeedPercentage) {
        SmartDashboard.putBoolean("Right Climber Activated", climbSpeedPercentage != 0);
        climbMotorLeft.set(climbSpeedPercentage);
    }

    public double getClimbRotations() {
        return climbMotorLeft.getEncoder().getPosition();
    }
}    

