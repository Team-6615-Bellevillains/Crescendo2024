package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class RotationSubsystem extends SubsystemBase {
    
    private CANSparkMax rotationMotor;

    public RotationSubsystem() {
        rotationMotor = new CANSparkMax(ArmConstants.kRotateMotorPort, MotorType.kBrushless);
    }

    public void setRotationSpeedPercentage(double rotationSpeedPercentage) {
        rotationMotor.set(rotationSpeedPercentage);
    }
}