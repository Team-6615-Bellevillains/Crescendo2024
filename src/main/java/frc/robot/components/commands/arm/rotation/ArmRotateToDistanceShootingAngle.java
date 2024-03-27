package frc.robot.components.commands.arm.rotation;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.components.subsystems.arm.RotationSubsystem;
import frc.robot.utils.controllers.TunableProfiledPIDController;

import static frc.robot.Constants.ArmConstants.RotationConstants;

public class ArmRotateToDistanceShootingAngle extends Command {

    private final RotationSubsystem rotationSubsystem;

    private final TunableProfiledPIDController radiansPositionController;

    public ArmRotateToDistanceShootingAngle(RotationSubsystem rotationSubsystem) {
        this.rotationSubsystem = rotationSubsystem;

        this.radiansPositionController = new TunableProfiledPIDController("distance arm rotation pid", RotationConstants.kPDistanceRotation,
                RotationConstants.kIDistanceRotation, RotationConstants.kDDistanceRotation,
                new TrapezoidProfile.Constraints(RotationConstants.kMaxDistanceRotationVelocityRadiansPerSecond,
                        RotationConstants.kMaxDistanceRotationAccelerationRadiansPerSecondSquared),
                rotationSubsystem::getRotationEncoderPositionInRadians);

        this.addRequirements(rotationSubsystem);
    }

    @Override
    public void initialize() {
        rotationSubsystem.deactivateHold();

        double armSetpointRadians = Units.degreesToRadians(RotationConstants.DISTANCE_SHOOTING_ANGLE_DEGREES);

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
        rotationSubsystem.setMotorVoltage(rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(), 0));
    }

//     @Override
//     public boolean isFinished() {
//         return Math.abs(rotationSubsystem.getRotationEncoderPositionInRadians() - radiansPositionController.getController()
//                 .getGoal().position) <= RotationConstants.DISTANCE_ROTATION_FINISHED_THRESHOLD_RADIANS;
//     }

}
