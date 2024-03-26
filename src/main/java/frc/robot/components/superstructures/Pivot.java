package frc.robot.components.superstructures;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.components.commands.arm.rotation.ArmRotateToDistanceShootingAngle;
import frc.robot.components.commands.arm.rotation.RotateArmAndHold;
import frc.robot.components.commands.arm.spin.*;
import frc.robot.components.subsystems.arm.RotationSubsystem;
import frc.robot.components.subsystems.arm.ShootingSubsystem;
import frc.robot.components.subsystems.arm.StorageSubsystem;
import frc.robot.utils.enums.Direction;

public class Pivot {
    private final RotationSubsystem rotationSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final ShootingSubsystem shootingSubsystem;

    public Pivot() {
        rotationSubsystem = new RotationSubsystem();
        storageSubsystem = new StorageSubsystem();
        shootingSubsystem = new ShootingSubsystem();
    }

    // Methods to create instances of shooting commands. Necessary for composition
    // in multiple places

    public ShootCmd speakerShooter() {
        return new ShootCmd(shootingSubsystem, Constants.ArmConstants.ShooterConstants.SHOOTING_SPEAKER_SHOOTER_VOLTAGE, storageSubsystem, Constants.ArmConstants.ShooterConstants.STORAGE_SPEAKER_SHOOTER_VOLTAGE);
    }

    public SpinUp spinUp() {
        return new SpinUp(shootingSubsystem, Constants.ArmConstants.ShooterConstants.SHOOTING_SPEAKER_SHOOTER_VOLTAGE);
    }

    public Command aimToSpeakerThenReset() {
        return spinUp()
                .alongWith(new ArmRotateToDistanceShootingAngle(rotationSubsystem))
                .andThen(new RotateArmAndHold(rotationSubsystem));
    }

    public Command intakeFromFloorThenReset() {
        return new RotateArmAndHold(rotationSubsystem)
                .andThen(new IntakeRingUntilCaptured(storageSubsystem, shootingSubsystem))
                .andThen(
                        new RotateArmAndHold(rotationSubsystem)
                                .alongWith(
                                        Commands.runOnce(() -> RobotContainer.state.setRumbling(true))
                                                .andThen(Commands.waitSeconds(Constants.OperatorConstants.RUMBLE_TIME_SECONDS))
                                                .andThen(Commands.runOnce(() -> RobotContainer.state.setRumbling(false)))
                                )
                );
    }

    public Command intakeFromSourceThenReset() {
        return new IntakeRingManual(storageSubsystem, shootingSubsystem)
                .alongWith(new ArmRotateToDistanceShootingAngle(rotationSubsystem))
                .andThen(new RotateArmAndHold(rotationSubsystem, Direction.UP));
    }

    public IntakeRingManual intakeRingManual() {
        return new IntakeRingManual(storageSubsystem, shootingSubsystem);
    }

    public RotateArmAndHold rotateArmAndHold() {
        return new RotateArmAndHold(rotationSubsystem);
    }

    public FeedNote feedNote() {
        return new FeedNote(storageSubsystem, Constants.ArmConstants.ShooterConstants.STORAGE_SPEAKER_SHOOTER_VOLTAGE);
    }

    public IntakeRingUntilCaptured intakeRingUntilCaptured() {
        return new IntakeRingUntilCaptured(storageSubsystem, shootingSubsystem);
    }

    public void activateArmHold() {
        rotationSubsystem.activateArmHold();
    }

}
