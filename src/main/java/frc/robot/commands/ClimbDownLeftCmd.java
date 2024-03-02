package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbLeftSubsystem;

public class ClimbDownLeftCmd extends Command {
    
    private ClimbLeftSubsystem climbLeftSubsystem;
    private DigitalInput leftDropInput;

    public ClimbDownLeftCmd(ClimbLeftSubsystem climbLeftSubsystem) {
        this.climbLeftSubsystem = climbLeftSubsystem;

        addRequirements(climbLeftSubsystem);
    }

    @Override
    public void initialize(){
        leftDropInput = new DigitalInput(5);
        climbLeftSubsystem.setClimbSpeedPercentage(-1.0); 
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Left down input", leftDropInput.get());
    }

    @Override
    public void end(boolean interrupted){
        leftDropInput.close();
        climbLeftSubsystem.setClimbSpeedPercentage(0); 
    }

    @Override
    public boolean isFinished() {
        return !leftDropInput.get();
    }
}
