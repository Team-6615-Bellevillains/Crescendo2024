package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class StorageSubsystem extends SubsystemBase {
    
    private CANSparkMax storageMotor;

    public StorageSubsystem() {
        storageMotor = new CANSparkMax(ArmConstants.kStorageMotorPort, MotorType.kBrushless);
    }

    public void setStorageSpeedPercentage(double storageSpeedPercentage) {
        storageMotor.set(storageSpeedPercentage);
    }

    public void setStorageSpeedVoltage(double storageSpeedVoltage) {
        storageMotor.setVoltage(storageSpeedVoltage);
    }
}
