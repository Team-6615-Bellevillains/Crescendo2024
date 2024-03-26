package frc.robot.components.commands.arm.spin;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ShooterConstants;
import frc.robot.components.subsystems.arm.ShootingSubsystem;
import frc.robot.components.subsystems.arm.StorageSubsystem;

import static frc.robot.Constants.ArmConstants.ShooterConstants;

public class FeedNote extends Command {

    private final StorageSubsystem storageSubsystem;
    private final double storageVoltage;
    private double startTime;

    public FeedNote(StorageSubsystem storageSubsystem, double storageVoltage) {
        this.storageSubsystem = storageSubsystem;
        this.storageVoltage = storageVoltage;

        addRequirements(storageSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        storageSubsystem.setStorageVoltage(storageVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        storageSubsystem.setStorageVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > startTime + ShooterConstants.LAUNCH_RUN_TIME-ShooterConstants.TIME_UNTIL_FEED;
    }
}
