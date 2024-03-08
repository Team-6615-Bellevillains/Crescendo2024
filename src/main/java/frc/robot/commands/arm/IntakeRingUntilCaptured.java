package frc.robot.commands.arm;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ShootingSubsystem;
import frc.robot.subsystems.arm.StorageSubsystem;

import static frc.robot.Constants.ArmConstants.ShooterConstants;

import java.util.function.BooleanSupplier;

public class IntakeRingUntilCaptured extends Command {

    private final StorageSubsystem storageSubsystem;
    private final ShootingSubsystem shootingSubsystem;
    private final BooleanSupplier slowWhenIntakingSupplier;
    private double startTime;
    private final double slowSpeed = 0.5;

    // default slow when intaking to false
    public IntakeRingUntilCaptured(StorageSubsystem storageSubsystem, ShootingSubsystem shootingSubsystem) {
        this.storageSubsystem = storageSubsystem;
        this.shootingSubsystem = shootingSubsystem;
        this.slowWhenIntakingSupplier = () -> false;

        addRequirements(storageSubsystem, shootingSubsystem);
    }

    public IntakeRingUntilCaptured(StorageSubsystem storageSubsystem, ShootingSubsystem shootingSubsystem, BooleanSupplier slowWhenIntakingSupplier) {
        this.storageSubsystem = storageSubsystem;
        this.shootingSubsystem = shootingSubsystem;
        this.slowWhenIntakingSupplier = slowWhenIntakingSupplier;

        addRequirements(storageSubsystem, shootingSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();

        storageSubsystem.setStorageVoltage(ShooterConstants.STORAGE_INTAKE_VOLTAGE);
        shootingSubsystem.setShootingVoltage(ShooterConstants.SHOOTING_INTAKE_VOLTAGE);

        RobotContainer.controlMultiplier = slowWhenIntakingSupplier.getAsBoolean() ? slowSpeed : 1.0;
    }

    @Override
    public void execute() { }

    @Override
    public void end(boolean interrupted) {
        storageSubsystem.setStorageVoltage(0);
        shootingSubsystem.setShootingVoltage(0);

        RobotContainer.controlMultiplier = 1.0;
    }

    @Override
    public boolean isFinished() {
        // return Timer.getFPGATimestamp() > startTime + ShooterConstants.INTAKE_SPIN_UP_DELAY_SECONDS
        return Math.abs(storageSubsystem.getOutputCurrent()) > 25;
    }

}