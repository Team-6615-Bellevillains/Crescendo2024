package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.RotationSubsystem;

public class RotateCmd extends Command {
    private RotationSubsystem rotationSubsystem;
    double rotationTime;
    double rotationSpeedPercentage;
    private double startTime;
    private boolean hasRotationFinished = false;



    public RotateCmd(RotationSubsystem rotationSubsystem, double rotationTime, double rotationSpeedPercentage){
        this.rotationSubsystem = rotationSubsystem;
        this.rotationTime = rotationTime;
        this.rotationSpeedPercentage = rotationSpeedPercentage;

        addRequirements(rotationSubsystem);
    }

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();

    }

    @Override
    public void execute(){
        while(Timer.getFPGATimestamp() < startTime + rotationTime) {
         rotationSubsystem.setRotationSpeedPercentage(rotationSpeedPercentage); //change once we test
        }
        if(Timer.getFPGATimestamp() > startTime + rotationTime){
            hasRotationFinished = true;
        }
    }
    //We will most likely have to not return the velocity to 0. We will need a small amount to keep it from falling down due to gravity
    //We may need to create separate shoot commands due to this
    @Override
    public void end(boolean interrupted){
        rotationSubsystem.setRotationSpeedPercentage(0); 
    }

    @Override 
    public boolean isFinished(){
        return hasRotationFinished;
    }
}
