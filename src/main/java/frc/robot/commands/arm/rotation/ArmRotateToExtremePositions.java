package frc.robot.commands.arm.rotation;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.RotationSubsystem;
import frc.robot.utils.enums.Direction;
import frc.robot.utils.controllers.TunableProfiledPIDController;

import static frc.robot.Constants.ArmConstants.RotationConstants;

public class ArmRotateToExtremePositions extends Command {

    private final RotationSubsystem rotationSubsystem;

    private final TunableProfiledPIDController radiansPositionController;

    private Direction holdDirection = null;

    public ArmRotateToExtremePositions(RotationSubsystem rotationSubsystem) {
        this.rotationSubsystem = rotationSubsystem;

        this.radiansPositionController = new TunableProfiledPIDController("rotation pid", RotationConstants.kPRotation,
                RotationConstants.kIRotation, RotationConstants.kDRotation,
                new TrapezoidProfile.Constraints(RotationConstants.kMaxRotationVelocityRadiansPerSecond,
                        RotationConstants.kMaxRotationAccelerationRadiansPerSecondSquared),
                rotationSubsystem::getRotationEncoderPositionInRadians);

        this.addRequirements(rotationSubsystem);
    }

    public ArmRotateToExtremePositions(RotationSubsystem rotationSubsystem, Direction holdDirection) {
        this(rotationSubsystem);
        
        this.holdDirection = holdDirection;
    }

    @Override
    public void initialize() {
        if (holdDirection == null) {
            rotationSubsystem.setArmHoldDirection(rotationSubsystem.getArmHoldDirection() == Direction.UP ? Direction.DOWN : Direction.UP);
        } else {
            rotationSubsystem.setArmHoldDirection(holdDirection);
        }

        rotationSubsystem.deactivateHold();

        double armSetpointRadians = Units.degreesToRadians(rotationSubsystem.getArmHoldDirection() == Direction.UP ?
                RotationConstants.SPEAKER_SHOOTING_ANGLE_DEGREES : RotationConstants.FLOOR_RESTING_ANGLE_DEGREES);

        SmartDashboard.putNumber("Arm Setpoint", armSetpointRadians);
        SmartDashboard.putString("Direction", rotationSubsystem.getArmHoldDirection() == Direction.UP ? "Speaker" : "Floor");

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
        SmartDashboard.putNumber("Rotation velocity at end",
                rotationSubsystem.getRotationEncoderVelocityInDegreesPerSec());
        SmartDashboard.putNumber("Rotation heartbeat at end", Timer.getFPGATimestamp());

        rotationSubsystem.activateArmHold();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rotationSubsystem.getRotationEncoderPositionInRadians() - radiansPositionController.getController()
                .getGoal().position) <= RotationConstants.ROTATION_FINISHED_THRESHOLD_RADIANS;
    }

}
