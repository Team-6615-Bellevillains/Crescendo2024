package frc.robot.components.commands.arm.spin;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ShooterConstants;
import frc.robot.State;
import frc.robot.components.subsystems.pivot.ShootingSubsystem;
import frc.robot.components.subsystems.pivot.StorageSubsystem;

public class IntakeRingUntilCaptured extends Command {

    private final StorageSubsystem storageSubsystem;
    private final ShootingSubsystem shootingSubsystem;
    private final Timer spinUpTimer = new Timer();

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

        State.getInstance().setIntakingState(true);

        spinUpTimer.restart();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        storageSubsystem.setStorageVoltage(0);
        shootingSubsystem.setShootingVoltage(0);

        State.getInstance().setIntakingState(false);
    }

    @Override
    public boolean isFinished() {
        return spinUpTimer.hasElapsed(ShooterConstants.NOTE_CAPTURED_SPIN_UP_WAIT)
                && Math.abs(storageSubsystem.getVelocityRPM()) < ShooterConstants.NOTE_CAPTURED_INTERNAL_ROLLER_STALL_RPM_THRESHOLD;
    }

}