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
    private double startTime;
    private boolean hasLauncherFinished = false;

    public ShootCmd(ShootingSubsystem shootingSubsystem, double shootSpeedPercentage, StorageSubsystem storageSubsystem) {
        this.shootingSubsystem = shootingSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.shootSpeedPercentage = shootSpeedPercentage;

        addRequirements(shootingSubsystem, storageSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }
    
    @Override
    public void execute() {
        while(Timer.getFPGATimestamp() < startTime + Constants.ArmConstants.LAUNCH_RUN_TIME) {
         shootingSubsystem.setShootingSpeedPercentage(shootSpeedPercentage); //change once we test
         if(Timer.getFPGATimestamp() > startTime + Constants.ArmConstants.LAUNCH_RUN_TIME - Constants.ArmConstants.LOAD_RUN_TIME){
            storageSubsystem.setStorageSpeedPercentage(0.1);
         }
        }
        if(Timer.getFPGATimestamp() > startTime + Constants.ArmConstants.LAUNCH_RUN_TIME){
            hasLauncherFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted){
        shootingSubsystem.setShootingSpeedPercentage(0); 
    }

    @Override 
    public boolean isFinished(){
        return hasLauncherFinished;
    }
}
