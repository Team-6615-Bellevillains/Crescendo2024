package frc.robot.components.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.ShooterConstants;

public class ShootingSubsystem extends SubsystemBase {

    private final CANSparkMax shootingMotor;

    public ShootingSubsystem() {
        shootingMotor = new CANSparkMax(ShooterConstants.kShootingMotorPort, MotorType.kBrushless);
        shootingMotor.setSmartCurrentLimit(80);
    }

    public void setShootingVoltage(double shootingVoltage) {
        shootingMotor.setVoltage(shootingVoltage);
    }
}