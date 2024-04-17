package frc.robot.components.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.ShooterConstants;

public class StorageSubsystem extends SubsystemBase {

    private final CANSparkMax storageMotor;
    private final RelativeEncoder storageEncoder;

    public StorageSubsystem() {
        storageMotor = new CANSparkMax(ShooterConstants.kStorageMotorPort, MotorType.kBrushless);
        storageMotor.setSmartCurrentLimit(80);

        storageEncoder = storageMotor.getEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Storage roller RPM", getVelocityRPM());
    }

    public void setStorageVoltage(double storageVoltage) {
        storageMotor.setVoltage(storageVoltage);
    }

    public double getOutputCurrent() {
        return storageMotor.getOutputCurrent();
    }

    public double getVelocityRPM() {
        return storageEncoder.getVelocity();
    }
}
