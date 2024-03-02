package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.ShooterConstants;

public class StorageSubsystem extends SubsystemBase {

    private final CANSparkMax storageMotor;

    public StorageSubsystem() {
        storageMotor = new CANSparkMax(ShooterConstants.kStorageMotorPort, MotorType.kBrushless);
    }

    public void setStorageVoltage(double storageVoltage) {
        storageMotor.setVoltage(storageVoltage);
    }

    public double getOutputCurrent() {
        return storageMotor.getOutputCurrent();
    }
}
