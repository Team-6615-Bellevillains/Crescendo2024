package frc.robot.components.commands.arm.rotation;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.components.subsystems.arm.RotationSubsystem;
import frc.robot.utils.enums.Direction;
import frc.robot.utils.controllers.TunableProfiledPIDController;

import static frc.robot.Constants.ArmConstants.RotationConstants;

public class RotateArmAndHold extends Command {

    private final RotationSubsystem rotationSubsystem;

    private final TunableProfiledPIDController radiansPositionController;

    private Direction holdDirection = null;

    public RotateArmAndHold(RotationSubsystem rotationSubsystem) {
        this.rotationSubsystem = rotationSubsystem;

        this.radiansPositionController = new TunableProfiledPIDController("rotation pid", RotationConstants.kPRotation,
                RotationConstants.kIRotation, RotationConstants.kDRotation,
                new TrapezoidProfile.Constraints(RotationConstants.kMaxRotationVelocityRadiansPerSecond,
                        RotationConstants.kMaxRotationAccelerationRadiansPerSecondSquared),
                rotationSubsystem::getRotationEncoderPositionInRadians);

        this.addRequirements(rotationSubsystem);
    }

    public RotateArmAndHold(RotationSubsystem rotationSubsystem, Direction holdDirection) {
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

        radiansPositionController.getController().setGoal(armSetpointRadians);
        radiansPositionController.getController().reset(rotationSubsystem.getRotationEncoderPositionInRadians());
    }

    @Override
    public void execute() {
        double pidOut = radiansPositionController.getController()
                .calculate(rotationSubsystem.getRotationEncoderPositionInRadians());
        double ffOut = rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(),
                radiansPositionController.getController().getSetpoint().velocity);

        double commandedVoltage = (pidOut + ffOut);

        rotationSubsystem.setMotorVoltage(commandedVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        rotationSubsystem.activateArmHold();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rotationSubsystem.getRotationEncoderPositionInRadians() - radiansPositionController.getController()
                .getGoal().position) <= RotationConstants.ROTATION_FINISHED_THRESHOLD_RADIANS;
    }

}
