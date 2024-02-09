package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ShootingSubsystem extends SubsystemBase {
    
    private CANSparkMax shootingMotor;

    public ShootingSubsystem() {
        shootingMotor = new CANSparkMax(ArmConstants.kShootingMotorPort, MotorType.kBrushless);
    }

    public void setShootingSpeedPercentage(double shootingSpeedPercentage) {
        shootingMotor.set(shootingSpeedPercentage);
    }
}