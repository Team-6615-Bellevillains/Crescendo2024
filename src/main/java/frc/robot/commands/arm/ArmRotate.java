package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.RotationSubsystem;
import frc.robot.utils.Direction;
import frc.robot.utils.TunableProfiledPIDController;

import static frc.robot.Constants.ArmConstants.RotationConstants;

public class ArmRotate extends Command {

    private final RotationSubsystem rotationSubsystem;

    private final TunableProfiledPIDController radiansPositionController;

    public ArmRotate(RotationSubsystem rotationSubsystem) {
        this.rotationSubsystem = rotationSubsystem;

        this.radiansPositionController = new TunableProfiledPIDController("rotation pid", RotationConstants.kPRotation,
                RotationConstants.kIRotation, RotationConstants.kDRotation,
                new TrapezoidProfile.Constraints(RotationConstants.kMaxRotationVelocityRadiansPerSecond,
                        RotationConstants.kMaxRotationAccelerationRadiansPerSecondSquared),
                rotationSubsystem::getRotationEncoderPositionInRadians);

        this.addRequirements(rotationSubsystem);
    }

    @Override
    public void initialize() {
        RotationSubsystem.armHoldDirection = RotationSubsystem.armHoldDirection == Direction.UP ? Direction.DOWN : Direction.UP;

        rotationSubsystem.deactivateHold();

        double armSetpointRadians = Units.degreesToRadians(RotationSubsystem.armHoldDirection == Direction.UP ?
                RotationConstants.SPEAKER_SHOOTING_ANGLE_DEGREES : RotationConstants.FLOOR_RESTING_ANGLE_DEGREES);

        SmartDashboard.putNumber("Arm Setpoint", armSetpointRadians);
        SmartDashboard.putString("Direction", RotationSubsystem.armHoldDirection == Direction.UP ? "Speaker" : "Floor");

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

        rotationSubsystem.activateArmHold();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rotationSubsystem.getRotationEncoderPositionInRadians() - radiansPositionController.getController()
                .getGoal().position) <= RotationConstants.ROTATION_FINISHED_THRESHOLD_RADIANS;
    }

}
