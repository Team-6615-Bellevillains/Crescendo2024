package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbRightSubsystem;

public class ClimbDownRightCmd extends Command {
    
    private ClimbRightSubsystem climbRightSubsystem;
   
    public ClimbDownRightCmd(ClimbRightSubsystem climbRightSubsystem) {
        this.climbRightSubsystem = climbRightSubsystem;

        addRequirements(climbRightSubsystem);
    }

    @Override
    public void initialize(){
   
    }

    @Override
    public void execute() {
         DigitalInput rightDropInput = new DigitalInput(3);
            while (rightDropInput.get() == true) {
            climbRightSubsystem.setClimbSpeedPercentage(-1.0); 
        System.out.println(rightDropInput.get());    
        }
        climbRightSubsystem.setClimbSpeedPercentage(0); 
        rightDropInput.close();
    }

    @Override
    public void end(boolean interrupted){
        climbRightSubsystem.setClimbSpeedPercentage(0); 
    }
}
