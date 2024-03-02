package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbLeftSubsystem;
import frc.robot.subsystems.arm.ShootingSubsystem;
import frc.robot.subsystems.arm.StorageSubsystem;

public class IntakeRingCmd extends Command {
    
    private StorageSubsystem storageSubsystem;
    private ShootingSubsystem shootingSubsystem;

    public IntakeRingCmd(StorageSubsystem storageSubsystem, ShootingSubsystem shootingSubsystem) {
        this.storageSubsystem = storageSubsystem;
        this.shootingSubsystem = shootingSubsystem;

        addRequirements(storageSubsystem, shootingSubsystem);
    }

    @Override
    public void initialize(){
        storageSubsystem.setStorageSpeedVoltage(-3.0); 
        shootingSubsystem.setShootingSpeedVoltage(6);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted){
        storageSubsystem.setStorageSpeedVoltage(0); 
        shootingSubsystem.setShootingSpeedVoltage(0);
    }

}