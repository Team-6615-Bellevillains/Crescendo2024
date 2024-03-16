package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.ShooterConstants;

public class StorageSubsystem extends SubsystemBase {

    private final CANSparkMax storageMotor;

    public StorageSubsystem() {
        storageMotor = new CANSparkMax(ShooterConstants.kStorageMotorPort, MotorType.kBrushless);
    }

    // TODO: Comment this out once intake threshold has been determined.
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Storage velocity", getVelocity());
    }

    public void setStorageVoltage(double storageVoltage) {
        storageMotor.setVoltage(storageVoltage);
    }

    public double getOutputCurrent() {
        return storageMotor.getOutputCurrent();
    }

    public double getVelocity() {
        return storageMotor.getEncoder().getVelocity();
    }
}
