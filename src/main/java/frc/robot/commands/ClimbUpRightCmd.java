package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbRightSubsystem;

public class ClimbUpRightCmd extends Command {
    
    private ClimbRightSubsystem climbRightSubsystem;
   
    public ClimbUpRightCmd(ClimbRightSubsystem climbRightSubsystem) {
        this.climbRightSubsystem = climbRightSubsystem;

        addRequirements(climbRightSubsystem);
    }

    @Override
    public void initialize(){
   
    }

    @Override
    public void execute() {
        DigitalInput rightClimbInput = new DigitalInput(2);
            while (rightClimbInput.get() == true) {
            climbRightSubsystem.setClimbSpeedPercentage(1.0); 
            System.out.println(rightClimbInput.get());
        }
        climbRightSubsystem.setClimbSpeedPercentage(0); 
        rightClimbInput.close();
        }

    @Override
    public void end(boolean interrupted){
        climbRightSubsystem.setClimbSpeedPercentage(0); 
    }
}
