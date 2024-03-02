package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbLeftSubsystem;

public class ClimbUpLeftCmd extends Command {
    
    private ClimbLeftSubsystem climbLeftSubsystem;
   
    public ClimbUpLeftCmd(ClimbLeftSubsystem climbLeftSubsystem) {
        this.climbLeftSubsystem = climbLeftSubsystem;

        addRequirements(climbLeftSubsystem);
    }

    @Override
    public void initialize(){
   
    }

    @Override
    public void execute() {
        DigitalInput leftClimbInput = new DigitalInput(4);
            while (leftClimbInput.get() == true) {
            climbLeftSubsystem.setClimbSpeedPercentage(1.0); 
            }
            climbLeftSubsystem.setClimbSpeedPercentage(0); 
            leftClimbInput.close();
    }

    @Override
    public void end(boolean interrupted){
        climbLeftSubsystem.setClimbSpeedPercentage(0); 
    }
}
