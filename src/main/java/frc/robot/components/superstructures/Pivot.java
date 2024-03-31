package frc.robot.components.superstructures;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.components.commands.arm.rotation.ArmRotateUsingJoystick;
import frc.robot.components.commands.arm.spin.*;
import frc.robot.components.subsystems.pivot.RotationSubsystem;
import frc.robot.components.subsystems.pivot.ShootingSubsystem;
import frc.robot.components.subsystems.pivot.StorageSubsystem;
import frc.robot.utils.enums.Direction;

import static frc.robot.Constants.ArmConstants.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.OperatorConstants;

public class Pivot {
    private final RotationSubsystem rotationSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final ShootingSubsystem shootingSubsystem;

    private Direction holdingDirection = Direction.UP;

    public Pivot() {
        rotationSubsystem = new RotationSubsystem();
        storageSubsystem = new StorageSubsystem();
        shootingSubsystem = new ShootingSubsystem();

        rotationSubsystem.setGoalPositionRadians(
                Units.degreesToRadians(RotationConstants.HOLD_UP_ANGLE_DEGREES));
    }

    // Methods to create instances of shooting commands. Necessary for composition
    // in multiple places

    public ShootCmd speakerShooter() {
        return new ShootCmd(shootingSubsystem, ShooterConstants.SHOOTING_SPEAKER_SHOOTER_VOLTAGE, storageSubsystem, ShooterConstants.STORAGE_SPEAKER_SHOOTER_VOLTAGE);
    }

    public SpinUp spinUp() {
        return new SpinUp(shootingSubsystem, ShooterConstants.SHOOTING_SPEAKER_SHOOTER_VOLTAGE);
    }

    public Command rotateArmWithJoystick(DoubleSupplier joystickInputSupplier) {
        return new ArmRotateUsingJoystick(rotationSubsystem, joystickInputSupplier);
    }

    public Command aimToSpeakerAndSpinUp() {
        return spinUp()
                .alongWith(setArmGoalPositionCommand(Units.degreesToRadians(RotationConstants.DISTANCE_SHOOTING_ANGLE_DEGREES)));
    }

    public Command intakeFromFloorThenReset() {
        return switchHoldDirectionAndHold()
                .andThen(new IntakeRingUntilCaptured(storageSubsystem, shootingSubsystem))
                .andThen(
                        switchHoldDirectionAndHold()
                                .alongWith(
                                        Commands.runOnce(() -> RobotContainer.state.setRumbling(true))
                                                .andThen(Commands.waitSeconds(OperatorConstants.RUMBLE_TIME_SECONDS))
                                                .andThen(Commands.runOnce(() -> RobotContainer.state.setRumbling(false)))
                                )
                );
    }

    public Command intakeFromSource() {
        return new IntakeRingManual(storageSubsystem, shootingSubsystem)
                .alongWith(setArmGoalPositionCommand(Units.degreesToRadians(RotationConstants.SOURCE_INTAKE_ANGLE_DEGREES)));
    }

    public IntakeRingManual intakeRingManual() {
        return new IntakeRingManual(storageSubsystem, shootingSubsystem);
    }

    public Command switchHoldDirectionAndHold() {
        return new FunctionalCommand(() -> {
                    holdingDirection = holdingDirection == Direction.UP ? Direction.DOWN : Direction.UP;
                    rotationSubsystem.setGoalPositionRadians(holdingDirection == Direction.UP
                                            ? Units.degreesToRadians(RotationConstants.HOLD_UP_ANGLE_DEGREES)
                                            : Units.degreesToRadians(RotationConstants.HOLD_DOWN_ANGLE_DEGREES));
        }, 
        () -> {}, interrupted -> { }, rotationSubsystem::atGoal, 
        rotationSubsystem);
    }

    public Command rotateArmAndHold(Direction direction) {
        return setArmGoalPositionCommand(direction == Direction.UP ?
                Units.degreesToRadians(RotationConstants.HOLD_UP_ANGLE_DEGREES)
                : Units.degreesToRadians(RotationConstants.HOLD_DOWN_ANGLE_DEGREES));
    }

    public Command setArmGoalPositionCommand(double goalPositionRadians) {
        return Commands.runOnce(() -> rotationSubsystem.setGoalPositionRadians(goalPositionRadians), rotationSubsystem);
    }

    public FeedNote feedNote() {
        return new FeedNote(storageSubsystem, ShooterConstants.STORAGE_SPEAKER_SHOOTER_VOLTAGE);
    }

    public IntakeRingUntilCaptured intakeRingUntilCaptured() {
        return new IntakeRingUntilCaptured(storageSubsystem, shootingSubsystem);
    }

    public void activateArmHold() {
        rotationSubsystem.hold();
    }

}
