package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ShootingSubsystem;
import frc.robot.subsystems.arm.StorageSubsystem;

public class ShootVoltageCmd extends Command {
   
    private ShootingSubsystem shootingSubsystem;
    private StorageSubsystem storageSubsystem;
    double shootSpeedVoltage;
    double storageSpeedVoltage;
    private double startTime;
    private boolean hasLauncherFinished = false;

    public ShootVoltageCmd(ShootingSubsystem shootingSubsystem, double shootSpeedPercentage, StorageSubsystem storageSubsystem, double storageSpeedPercentage) {
        this.shootingSubsystem = shootingSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.shootSpeedVoltage = shootSpeedPercentage;
        this.storageSpeedVoltage = storageSpeedPercentage;

        addRequirements(shootingSubsystem, storageSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }
    
    @Override
    public void execute() {
        while(Timer.getFPGATimestamp() < startTime + Constants.ArmConstants.LAUNCH_RUN_TIME) {
        shootingSubsystem.setShootingSpeedVoltage(shootSpeedVoltage);
        storageSubsystem.setStorageSpeedVoltage(storageSpeedVoltage);
        }
        if(Timer.getFPGATimestamp() > startTime + Constants.ArmConstants.LAUNCH_RUN_TIME){
            hasLauncherFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted){
        shootingSubsystem.setShootingSpeedPercentage(0); 
        storageSubsystem.setStorageSpeedPercentage(0); 

    }

    @Override 
    public boolean isFinished(){
        return hasLauncherFinished;
    }
}
