package frc.robot.components.commands.arm.spin;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.components.subsystems.pivot.ShootingSubsystem;
import frc.robot.components.subsystems.pivot.StorageSubsystem;

import static frc.robot.Constants.ArmConstants.ShooterConstants;

public class ShootCmd extends Command {

    private final ShootingSubsystem shootingSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final double shootVoltage;
    private final double storageVoltage;
    private Timer shootTimer;

    public ShootCmd(ShootingSubsystem shootingSubsystem, double shootVoltage, StorageSubsystem storageSubsystem, double storageVoltage) {
        this.shootingSubsystem = shootingSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.shootVoltage = shootVoltage;
        this.storageVoltage = storageVoltage;

        addRequirements(shootingSubsystem, storageSubsystem);
    }

    @Override
    public void initialize() {
        shootTimer.restart();

        shootingSubsystem.setShootingVoltage(shootVoltage);
    }

    @Override
    public void execute() {
        if (shootTimer.hasElapsed(ShooterConstants.TIME_UNTIL_FEED)) {
            storageSubsystem.setStorageVoltage(storageVoltage);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shootingSubsystem.setShootingVoltage(0);
        storageSubsystem.setStorageVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return shootTimer.hasElapsed(ShooterConstants.LAUNCH_RUN_TIME);
    }
}
