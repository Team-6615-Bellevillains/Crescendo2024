package frc.robot.components.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.controllers.TunableProfiledPIDController;
import frc.robot.utils.controllers.TunableArmFeedforward;

import static frc.robot.Constants.ArmConstants.RotationConstants;

public class RotationSubsystem extends SubsystemBase {

    private final CANSparkMax rotationMotor;
    private final RelativeEncoder rotationEncoder;

    private final TunableProfiledPIDController rotationProfiledPID;
    private final TunableArmFeedforward rotationFeedforward;

    private double goalPositionRadians = Units.degreesToRadians(RotationConstants.HOLD_UP_ANGLE_DEGREES);
    private boolean holding = false;

    public RotationSubsystem() {
        rotationMotor = new CANSparkMax(RotationConstants.kRotateMotorPort, MotorType.kBrushless);
        rotationEncoder = rotationMotor.getAlternateEncoder(4096);

        rotationMotor.setInverted(true);
        rotationEncoder.setPositionConversionFactor(RotationConstants.ENCODER_READING_TO_ANGLE_CONVERSION_FACTOR);
        rotationEncoder.setVelocityConversionFactor(RotationConstants.ENCODER_READING_TO_ANGLE_CONVERSION_FACTOR / 60.0);
        rotationEncoder.setPosition(RotationConstants.HOLD_UP_ANGLE_DEGREES);

        // Make further configuration calls on the rotation motor non-blocking.
        // That is, the program will continue to run without waiting for the motor to respond.
        // Important to not have a huge delay when setting holding current limit
        rotationMotor.setSmartCurrentLimit(80);
        rotationMotor.setCANTimeout(0);

        rotationProfiledPID = new TunableProfiledPIDController("rotation pid", RotationConstants.kPRotation,
                RotationConstants.kIRotation, RotationConstants.kDRotation,
                new TrapezoidProfile.Constraints(RotationConstants.kMaxRotationVelocityRadiansPerSecond,
                        RotationConstants.kMaxRotationAccelerationRadiansPerSecondSquared),
                this::getRotationEncoderPositionInRadians);
        rotationProfiledPID.getController().setTolerance(RotationConstants.ROTATION_PROFILEDPID_POSITION_TOLERANCE_RADIANS, RotationConstants.ROTATION_PROFILEDPID_VELOCITY_TOLERANCE_RADS_PER_SECOND);

        rotationFeedforward = new TunableArmFeedforward("rotation", RotationConstants.kSRotation,
                RotationConstants.kGRotation, RotationConstants.kVRotation, RotationConstants.kARotation);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("At goal?", rotationProfiledPID.getController().atGoal());
        if ((goalPositionRadians == Units.degreesToRadians(RotationConstants.HOLD_UP_ANGLE_DEGREES)
                || goalPositionRadians == Units.degreesToRadians(RotationConstants.HOLD_DOWN_ANGLE_DEGREES))
                && rotationProfiledPID.getController().atGoal()) {
            hold();
            return;
        }

        double pidOut = rotationProfiledPID.getController()
                .calculate(getRotationEncoderPositionInRadians());
        double ffOut = calculateFeedforward(getRotationEncoderPositionInRadians(),
                rotationProfiledPID.getController().getSetpoint().velocity);

        double commandedVoltage = (pidOut + ffOut);

        
        SmartDashboard.putNumber("Trap velo", rotationProfiledPID.getController().getSetpoint().velocity);
        SmartDashboard.putNumber("Actual arm position degrees", getRotationEncoderPositionInDegrees());
        SmartDashboard.putNumber("Actual arm position radians", getRotationEncoderPositionInRadians());
        SmartDashboard.putNumber("Actual arm rotation", getRotationEncoderVelocityInRadsPerSec());
        SmartDashboard.putNumber("Commanded voltage", commandedVoltage);
        SmartDashboard.putNumber("periodic", Timer.getFPGATimestamp());
        setMotorVoltage(commandedVoltage);
    }

    public double calculateFeedforward(double currentPositionRadians, double desiredVelocityRadians) {
        return rotationFeedforward.getController().calculate(currentPositionRadians, desiredVelocityRadians);
    }

    public void setMotorVoltage(double voltage) {
        SmartDashboard.putNumber("NOW Rotation voltage", voltage);
        SmartDashboard.putNumber("NOW Rotation Timestamp", Timer.getFPGATimestamp());
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

    public void setGoalPositionRadians(double newGoalPositionRadians) {
        deactivateHold();

        this.goalPositionRadians = newGoalPositionRadians;
        SmartDashboard.putNumber("Goal", goalPositionRadians);
        rotationProfiledPID.getController().setGoal(newGoalPositionRadians);
    }

    public void hold() {
        if (holding) {
            return;
        }

        rotationMotor.setSmartCurrentLimit(RotationConstants.HOLDING_ANGLE_CURRENT_LIMIT);

        int holdingVoltageSign = goalPositionRadians == RotationConstants.HOLD_UP_ANGLE_DEGREES ? 1 : -1;
        setMotorVoltage(holdingVoltageSign * RotationConstants.HOLDING_ANGLE_VOLTAGE);

        holding = true;
    }

    public void deactivateHold() {
        holding = false;

        rotationMotor.setSmartCurrentLimit(RotationConstants.REGULAR_CURRENT_LIMIT);

        setMotorVoltage(0);
    }

}