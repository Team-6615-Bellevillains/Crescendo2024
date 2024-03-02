package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbLeftSubsystem;

public class ClimbUpLeftCmd extends Command {
    
    private ClimbLeftSubsystem climbLeftSubsystem;
    private DigitalInput leftClimbInput;

    public ClimbUpLeftCmd(ClimbLeftSubsystem climbLeftSubsystem) {
        this.climbLeftSubsystem = climbLeftSubsystem;

        addRequirements(climbLeftSubsystem);
    }

    @Override
    public void initialize(){
        leftClimbInput = new DigitalInput(4);
        climbLeftSubsystem.setClimbSpeedPercentage(1.0); 
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Left Up input", leftClimbInput.get());
    }

    @Override
    public void end(boolean interrupted){
        leftClimbInput.close();
        climbLeftSubsystem.setClimbSpeedPercentage(0); 
    }

    @Override
    public boolean isFinished() {
        return !leftClimbInput.get();
    }
}
