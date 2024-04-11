package frc.robot.components.commands.arm.spin;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ShooterConstants;
import frc.robot.components.subsystems.pivot.StorageSubsystem;

public class FeedNote extends Command {

    private final StorageSubsystem storageSubsystem;
    private final double storageVoltage;
    private Timer feedTimer = new Timer();

    public FeedNote(StorageSubsystem storageSubsystem, double storageVoltage) {
        this.storageSubsystem = storageSubsystem;
        this.storageVoltage = storageVoltage;

        addRequirements(storageSubsystem);
    }

    @Override
    public void initialize() {
        feedTimer.restart();
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
        return feedTimer.hasElapsed(ShooterConstants.LAUNCH_RUN_TIME-ShooterConstants.TIME_UNTIL_FEED);
    }
}
