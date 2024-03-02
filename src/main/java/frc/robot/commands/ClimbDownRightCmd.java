package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbRightSubsystem;

public class ClimbDownRightCmd extends Command {
    
    private ClimbRightSubsystem climbRightSubsystem;
    private DigitalInput rightDropInput;

    public ClimbDownRightCmd(ClimbRightSubsystem climbRightSubsystem) {
        this.climbRightSubsystem = climbRightSubsystem;

        addRequirements(climbRightSubsystem);
    }

    @Override
    public void initialize(){
        rightDropInput = new DigitalInput(3);
        climbRightSubsystem.setClimbSpeedPercentage(-1.0); 
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Right down input", rightDropInput.get());
    }

    @Override
    public void end(boolean interrupted){
        rightDropInput.close();
        climbRightSubsystem.setClimbSpeedPercentage(0); 
    }

    @Override
    public boolean isFinished() {
        return !rightDropInput.get();
    }
}
