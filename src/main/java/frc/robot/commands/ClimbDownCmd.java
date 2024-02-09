package frc.robot.commands;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDownCmd extends Command {
    
    private ClimbSubsystem climbSubsystem;
    public PWM climbDownRightPwm;
    public PWM climbDownLeftPwm;

    public ClimbDownCmd(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize(){
       climbDownRightPwm.setPosition(1);
       climbDownLeftPwm.setPosition(1);
    }

    @Override
    public void execute() {
        if(climbDownRightPwm.getPosition() > 0){
            climbSubsystem.setClimbSpeedPercentage(-0.25); //change once we test
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
