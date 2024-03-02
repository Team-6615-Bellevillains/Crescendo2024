package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.RotationSubsystem;
import frc.robot.utils.TunableProfiledPIDController;

public class RotateToSpecificAngle extends Command {

    private final RotationSubsystem rotationSubsystem;

    private final TunableProfiledPIDController radiansPositionController;
    private final BooleanSupplier directionSupplier;
    private boolean isHoldingSpeakerAngle;

    public RotateToSpecificAngle(RotationSubsystem rotationSubsystem, BooleanSupplier directionSupplier) {
        this.rotationSubsystem = rotationSubsystem;
        this.directionSupplier = directionSupplier;

        this.radiansPositionController = new TunableProfiledPIDController("rotation pid", ArmConstants.kPRotation,
                ArmConstants.kIRotation, ArmConstants.kDRotation,
                new TrapezoidProfile.Constraints(ArmConstants.kMaxRotationVelocityRadiansPerSecond,
                        ArmConstants.kMaxRotationAccelerationRadiansPerSecondSquared),
                rotationSubsystem::getRotationEncoderPositionInRadians);

        this.addRequirements(rotationSubsystem);
    }

    @Override
    public void initialize() {
        rotationSubsystem.deactivateHoldingCurrentLimit();

        isHoldingSpeakerAngle = directionSupplier.getAsBoolean();
        double armSetpointRadians = Units.degreesToRadians(isHoldingSpeakerAngle ? 
                ArmConstants.SPEAKER_SHOOTING_ANGLE_DEGREES : ArmConstants.FLOOR_RESTING_ANGLE_DEGREES);

        SmartDashboard.putNumber("Arm Setpoint", armSetpointRadians);
        SmartDashboard.putString("Direction", isHoldingSpeakerAngle ? "Speaker" : "Floor");

        radiansPositionController.getController().setGoal(armSetpointRadians);
        radiansPositionController.getController().reset(rotationSubsystem.getRotationEncoderPositionInRadians());
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Rotation Setpoint heartbeat", Timer.getFPGATimestamp());
        SmartDashboard.putNumber("Rotation Setpoint Goal Position",
                radiansPositionController.getController().getGoal().position);

        double pidOut = radiansPositionController.getController()
                .calculate(rotationSubsystem.getRotationEncoderPositionInRadians());
        double ffOut = rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(),
                radiansPositionController.getController().getSetpoint().velocity);
       
        double commandedVoltage = (pidOut + ffOut);

        SmartDashboard.putNumber("Rotation pidOutput", pidOut);
        SmartDashboard.putNumber("Rotation ffout", ffOut);
        SmartDashboard.putNumber("Rotation commanded voltage", commandedVoltage);

        rotationSubsystem.setMotorVoltage(commandedVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Rotation position at end", rotationSubsystem.getRotationEncoderPositionInDegrees());
        SmartDashboard.putNumber("Rotation position at end", rotationSubsystem.getRotationEncoderPositionInDegrees());
        SmartDashboard.putNumber("Rotation velocity at end",
                rotationSubsystem.getRotationEncoderVelocityInDegreesPerSec());
        SmartDashboard.putNumber("Rotation heartbeat at end", Timer.getFPGATimestamp());

        rotationSubsystem.activateHoldingCurrentLimit();

        int holdingVoltageSign = isHoldingSpeakerAngle ? 1 : -1;
        rotationSubsystem.setMotorVoltage(holdingVoltageSign * ArmConstants.HOLDING_ANGLE_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rotationSubsystem.getRotationEncoderPositionInRadians() - radiansPositionController.getController()
                .getGoal().position) <= ArmConstants.ROTATION_FINISHED_THRESHOLD_RADIANS;
    }

}
