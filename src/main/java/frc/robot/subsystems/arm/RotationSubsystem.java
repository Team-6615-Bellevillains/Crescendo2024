package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.TunableArmFeedforward;

public class RotationSubsystem extends SubsystemBase {

    private CANSparkMax rotationMotor;
    private RelativeEncoder rotationEncoder;

    private TunableArmFeedforward rotationFeedforward = new TunableArmFeedforward("rotation", ArmConstants.kSRotation,
            ArmConstants.kGRotation, ArmConstants.kVRotation, ArmConstants.kARotation);

    public RotationSubsystem() {
        rotationMotor = new CANSparkMax(ArmConstants.kRotateMotorPort, MotorType.kBrushless);
        rotationEncoder = rotationMotor.getEncoder();

        rotationMotor.setInverted(true);
        rotationEncoder.setPositionConversionFactor(ArmConstants.ENCODER_READING_TO_ANGLE_CONVERSION_FACTOR);
        rotationEncoder.setVelocityConversionFactor(ArmConstants.ENCODER_READING_TO_ANGLE_CONVERSION_FACTOR / 60.0);
        rotationEncoder.setPosition(ArmConstants.FLOOR_RESTING_ANGLE_DEGREES);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Rotation measurement", rotationEncoder.getPosition());
        SmartDashboard.putNumber("Rotation measurement [Heartbeat]", Timer.getFPGATimestamp());
    }

    public double calculateFeedforward(double currentPositionRadians, double desiredVelocityRadians) {
       return rotationFeedforward.getController().calculate(currentPositionRadians, desiredVelocityRadians);
    }

    public void setMotorVoltage(double voltage) {
        SmartDashboard.putNumber("Rotation voltage", voltage);
        SmartDashboard.putNumber("Rotation voltage [Heartbeat]", Timer.getFPGATimestamp());

        rotationMotor.setVoltage(voltage);
    }

    public void resetEncoderToSpeakerPosition() {
        rotationEncoder.setPosition(ArmConstants.SPEAKER_SHOOTING_ANGLE_DEGREES);
    }

    public void resetEncoderToFloorRestPosition() {
        rotationEncoder.setPosition(ArmConstants.FLOOR_RESTING_ANGLE_DEGREES);
    }

    public double getRotationEncoderPositionInDegrees() {
        return rotationEncoder.getPosition();
    }
    
    public double getRotationEncoderPositionInRadians() {
        return Units.degreesToRadians(getRotationEncoderPositionInDegrees());
    }

    public double getRotationEncoderVelocityInDegreesPerSec() {
        return rotationEncoder.getVelocity();
    }
    
    public double getRotationEncoderVelocityInRadsPerSec() {
        return Units.degreesToRadians(getRotationEncoderVelocityInDegreesPerSec());
    }
}