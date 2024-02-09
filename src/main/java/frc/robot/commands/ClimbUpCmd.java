package frc.robot.commands;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbUpCmd extends Command {
    
    private ClimbSubsystem climbSubsystem;
    public PWM climbUpRightPwm;
    public PWM climbUpLeftPwm;


    public ClimbUpCmd(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize(){
       climbUpRightPwm.setPosition(0);
       climbUpLeftPwm.setPosition(0);
    }

    @Override
    public void execute() {
        if(climbUpRightPwm.getPosition() < 1){
            climbSubsystem.setClimbSpeedPercentage(0.25); //change once we test
        }
        else {
            climbSubsystem.setClimbSpeedPercentage(0); 

        }
    }

    @Override
    public void end(boolean interrupted){
        climbSubsystem.setClimbSpeedPercentage(0); 
    }
}
