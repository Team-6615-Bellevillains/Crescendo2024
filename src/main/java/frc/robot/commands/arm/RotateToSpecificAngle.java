package frc.robot.commands.arm;

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

    private final TunableProfiledPIDController positionController;

    public RotateToSpecificAngle(RotationSubsystem rotationSubsystem, double desiredAngleRadians) {
        this.rotationSubsystem = rotationSubsystem;

        this.positionController = new TunableProfiledPIDController("rotation pid", ArmConstants.kPRotation,
                ArmConstants.kIRotation, ArmConstants.kDRotation,
                new TrapezoidProfile.Constraints(ArmConstants.kMaxRotationVelocityRadiansPerSecond,
                        ArmConstants.kMaxRotationAccelerationRadiansPerSecondSquared),
                rotationSubsystem::getRotationEncoderPositionInRadians);

        this.positionController.getController().setGoal(desiredAngleRadians);

        this.addRequirements(rotationSubsystem);
    }

    @Override
    public void initialize() {
        positionController.getController().reset(rotationSubsystem.getRotationEncoderPositionInRadians());
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Rotation Setpoint heartbeat", Timer.getFPGATimestamp());
        SmartDashboard.putNumber("Rotation Setpoint Goal Position",
                positionController.getController().getGoal().position);

        double pidOut = positionController.getController()
                .calculate(rotationSubsystem.getRotationEncoderPositionInRadians());
        double ffOut = rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(),
                positionController.getController().getSetpoint().velocity);
        // double commandedVoltage = pidOut + ffOut;
        double commandedVoltage = ffOut;

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

        // rotationSubsystem.setMotorVoltage(
        //         rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(), 0));
        rotationSubsystem.setMotorVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rotationSubsystem.getRotationEncoderPositionInRadians() - positionController.getController()
                .getGoal().position) <= ArmConstants.ROTATION_FINISHED_THRESHOLD_RADIANS;
    }

}
