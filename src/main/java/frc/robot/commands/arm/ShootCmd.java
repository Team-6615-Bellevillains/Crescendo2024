package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ShootingSubsystem;
import frc.robot.subsystems.arm.StorageSubsystem;

public class ShootCmd extends Command {
   
    private ShootingSubsystem shootingSubsystem;
    private StorageSubsystem storageSubsystem;
    double shootSpeedPercentage;
    double storageSpeedPercentage;
    private double startTime;
    private boolean hasLauncherFinished = false;

    public ShootCmd(ShootingSubsystem shootingSubsystem, double shootSpeedPercentage, StorageSubsystem storageSubsystem, double storageSpeedPercentage) {
        this.shootingSubsystem = shootingSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.shootSpeedPercentage = shootSpeedPercentage;
        this.storageSpeedPercentage = storageSpeedPercentage;

        addRequirements(shootingSubsystem, storageSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }
    
    @Override
    public void execute() {
        while(Timer.getFPGATimestamp() < startTime + Constants.ArmConstants.LAUNCH_RUN_TIME) {
        shootingSubsystem.setShootingSpeedPercentage(shootSpeedPercentage); 
        storageSubsystem.setStorageSpeedPercentage(storageSpeedPercentage);
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
