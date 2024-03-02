package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TestClimbSensorsCmd extends Command{
    
    DigitalInput rightDrop, rightClimb, leftDrop, leftClimb;
    

    @Override
    public void initialize() {
        rightDrop = new DigitalInput(3);
        rightClimb = new DigitalInput(2);
        leftDrop = new DigitalInput(5);
        leftClimb = new DigitalInput(4);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("rightDrop", rightDrop.get());        
        SmartDashboard.putNumber("heartbeat", Timer.getFPGATimestamp());
        SmartDashboard.putBoolean("rightClimb", rightClimb.get());
        SmartDashboard.putBoolean("leftDrop", leftDrop.get());
        SmartDashboard.putBoolean("leftClimb", leftClimb.get());
    }

    @Override
    public void end(boolean interrupted) {
        rightDrop.close();
        rightClimb.close();
        leftDrop.close();
        leftClimb.close();
    }

}
