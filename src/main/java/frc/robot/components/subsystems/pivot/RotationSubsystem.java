package frc.robot.components.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.enums.Direction;
import frc.robot.utils.controllers.TunableArmFeedforward;

import static frc.robot.Constants.ArmConstants.RotationConstants;

public class RotationSubsystem extends SubsystemBase {

    private Direction armHoldDirection = Direction.UP;
    private final CANSparkMax rotationMotor;
    private final RelativeEncoder rotationEncoder;

    private final TunableArmFeedforward rotationFeedforward = new TunableArmFeedforward("rotation", RotationConstants.kSRotation,
            RotationConstants.kGRotation, RotationConstants.kVRotation, RotationConstants.kARotation);

    public RotationSubsystem() {
        rotationMotor = new CANSparkMax(RotationConstants.kRotateMotorPort, MotorType.kBrushless);
        rotationEncoder = rotationMotor.getAlternateEncoder(4096);

        rotationMotor.setInverted(true);
        rotationEncoder.setPositionConversionFactor(RotationConstants.ENCODER_READING_TO_ANGLE_CONVERSION_FACTOR);
        rotationEncoder.setVelocityConversionFactor(RotationConstants.ENCODER_READING_TO_ANGLE_CONVERSION_FACTOR / 60.0);
        rotationEncoder.setPosition(RotationConstants.SPEAKER_SHOOTING_ANGLE_DEGREES);
    }

    public double calculateFeedforward(double currentPositionRadians, double desiredVelocityRadians) {
        return rotationFeedforward.getController().calculate(currentPositionRadians, desiredVelocityRadians);
    }

    public void setMotorVoltage(double voltage) {
        rotationMotor.setVoltage(voltage);
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

    public Direction getArmHoldDirection() {
        return armHoldDirection;
    }

    public void setArmHoldDirection(Direction newArmHoldDirection) {
        this.armHoldDirection = newArmHoldDirection;
    }

    public void activateArmHold() {
        rotationMotor.setSmartCurrentLimit(RotationConstants.HOLDING_ANGLE_CURRENT_LIMIT);

        int holdingVoltageSign = armHoldDirection == Direction.UP ? 1 : -1;
        setMotorVoltage(holdingVoltageSign * RotationConstants.HOLDING_ANGLE_VOLTAGE);
    }

    public void deactivateHold() {
        rotationMotor.setSmartCurrentLimit(RotationConstants.REGULAR_CURRENT_LIMIT);

        setMotorVoltage(0);
    }
}