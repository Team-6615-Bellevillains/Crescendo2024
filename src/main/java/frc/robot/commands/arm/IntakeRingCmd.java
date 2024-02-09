package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
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
    public void execute() {
        DigitalInput intakeInput = new DigitalInput(1); //change once wired
        while (intakeInput.get() == false) {
        storageSubsystem.setStorageSpeedPercentage(-0.1); //change once we test
        shootingSubsystem.setShootingSpeedPercentage(-0.1); //change once we test
        }
    }

    @Override
    public void end(boolean interrupted){
        storageSubsystem.setStorageSpeedPercentage(0); 
        shootingSubsystem.setShootingSpeedPercentage(0); 
    }
}
