package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbLeftSubsystem;

public class ClimbDownLeftCmd extends Command {
    
    private ClimbLeftSubsystem climbLeftSubsystem;
   
    public ClimbDownLeftCmd(ClimbLeftSubsystem climbLeftSubsystem) {
        this.climbLeftSubsystem = climbLeftSubsystem;

        addRequirements(climbLeftSubsystem);
    }

    @Override
    public void initialize(){
   
    }

    @Override
    public void execute() {
         DigitalInput leftDropInput = new DigitalInput(5);
            while (leftDropInput.get() == true) {
            climbLeftSubsystem.setClimbSpeedPercentage(-1.0); 
        } 
        climbLeftSubsystem.setClimbSpeedPercentage(0); 
        leftDropInput.close();
    }

    @Override
    public void end(boolean interrupted){
        climbLeftSubsystem.setClimbSpeedPercentage(0); 
    }
}
