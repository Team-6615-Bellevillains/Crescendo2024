package frc.robot.components.commands.arm.spin;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.components.subsystems.arm.ShootingSubsystem;
import frc.robot.components.subsystems.arm.StorageSubsystem;
import frc.robot.components.subsystems.drive.SwerveSubsystem;

import java.util.function.BooleanSupplier;

public class IntakeRingUntilCaptured extends Command {

    private final StorageSubsystem storageSubsystem;
    private final ShootingSubsystem shootingSubsystem;

    // default slow when intaking to false
    public IntakeRingUntilCaptured(StorageSubsystem storageSubsystem, ShootingSubsystem shootingSubsystem) {
        this.storageSubsystem = storageSubsystem;
        this.shootingSubsystem = shootingSubsystem;

        addRequirements(storageSubsystem, shootingSubsystem);
    }

    @Override
    public void initialize() {
        storageSubsystem.setStorageVoltage(ShooterConstants.STORAGE_INTAKE_VOLTAGE);
        shootingSubsystem.setShootingVoltage(ShooterConstants.SHOOTING_INTAKE_VOLTAGE);

        RobotContainer.state.setIntakingState(true);
    }

    @Override
    public void execute() { }

    @Override
    public void end(boolean interrupted) {
        storageSubsystem.setStorageVoltage(0);
        shootingSubsystem.setShootingVoltage(0);

        RobotContainer.state.setIntakingState(false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(storageSubsystem.getOutputCurrent()) > ShooterConstants.NOTE_CAPTURED_STALL_CURRENT_THRESHOLD;
    }

}