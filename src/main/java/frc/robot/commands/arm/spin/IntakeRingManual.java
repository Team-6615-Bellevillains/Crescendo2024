package frc.robot.commands.arm.spin;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ShootingSubsystem;
import frc.robot.subsystems.arm.StorageSubsystem;

import static frc.robot.Constants.ArmConstants.ShooterConstants;

public class IntakeRingManual extends Command {

    private final StorageSubsystem storageSubsystem;
    private final ShootingSubsystem shootingSubsystem;

    public IntakeRingManual(StorageSubsystem storageSubsystem, ShootingSubsystem shootingSubsystem) {
        this.storageSubsystem = storageSubsystem;
        this.shootingSubsystem = shootingSubsystem;

        addRequirements(storageSubsystem, shootingSubsystem);
    }

    @Override
    public void initialize() {
        storageSubsystem.setStorageVoltage(ShooterConstants.STORAGE_INTAKE_VOLTAGE);
        shootingSubsystem.setShootingVoltage(ShooterConstants.SHOOTING_INTAKE_VOLTAGE);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        storageSubsystem.setStorageVoltage(0);
        shootingSubsystem.setShootingVoltage(0);
    }

}