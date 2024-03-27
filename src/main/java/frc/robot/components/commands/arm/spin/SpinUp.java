package frc.robot.components.commands.arm.spin;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.components.subsystems.pivot.ShootingSubsystem;

public class SpinUp extends Command {

    private final ShootingSubsystem shootingSubsystem;
    private final double shootVoltage;

    public SpinUp(ShootingSubsystem shootingSubsystem, double shootVoltage) {
        this.shootingSubsystem = shootingSubsystem;
        this.shootVoltage = shootVoltage;

        addRequirements(shootingSubsystem);
    }

    @Override
    public void initialize() {
        shootingSubsystem.setShootingVoltage(shootVoltage);
    }

    @Override
    public void execute() { }

    @Override
    public void end(boolean interrupted) {
        shootingSubsystem.setShootingVoltage(0);
    }
}
