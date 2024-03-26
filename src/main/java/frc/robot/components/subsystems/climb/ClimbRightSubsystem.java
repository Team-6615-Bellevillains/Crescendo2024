package frc.robot.components.subsystems.climb;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbRightSubsystem extends SubsystemBase {

    private final CANSparkMax climbMotorRight;

    public ClimbRightSubsystem() {
        climbMotorRight = new CANSparkMax(ClimbConstants.kClimbMotorRightPort, MotorType.kBrushless);
        climbMotorRight.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Climb Right Rotations", getClimbRotations());
    }

    public void setClimbSpeedPercentage(double climbSpeedPercentage) {
        SmartDashboard.putBoolean("Right Climber Activated", climbSpeedPercentage != 0);
        climbMotorRight.set(climbSpeedPercentage);
    }

    public double getClimbRotations() {
        return climbMotorRight.getEncoder().getPosition();
    }
}    