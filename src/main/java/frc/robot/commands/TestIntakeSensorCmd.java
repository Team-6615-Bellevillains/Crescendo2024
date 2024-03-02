package frc.robot.commands;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TestIntakeSensorCmd extends Command{
    
    AnalogInput intake;
    

    @Override
    public void initialize() {
        intake = new AnalogInput(0);
        
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("intake", intake.getVoltage());        
        SmartDashboard.putNumber("heartbeat", Timer.getFPGATimestamp());
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.close();
        
    }

}
