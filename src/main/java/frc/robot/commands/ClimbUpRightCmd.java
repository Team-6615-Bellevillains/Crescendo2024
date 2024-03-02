package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbRightSubsystem;

public class ClimbUpRightCmd extends Command {
    
    private ClimbRightSubsystem climbRightSubsystem;
    private DigitalInput RightClimbInput;

    public ClimbUpRightCmd(ClimbRightSubsystem climbRightSubsystem) {
        this.climbRightSubsystem = climbRightSubsystem;

        addRequirements(climbRightSubsystem);
    }

    @Override
    public void initialize(){
        RightClimbInput = new DigitalInput(2);
        climbRightSubsystem.setClimbSpeedPercentage(1.0); 
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Right Up input", RightClimbInput.get());
    }

    @Override
    public void end(boolean interrupted){
        RightClimbInput.close();
        climbRightSubsystem.setClimbSpeedPercentage(0); 
    }

    @Override
    public boolean isFinished() {
        return !RightClimbInput.get();
    }
}
