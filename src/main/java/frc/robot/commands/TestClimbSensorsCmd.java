package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.ClimbConstants;

public class TestClimbSensorsCmd extends Command {

    DigitalInput rightDrop, rightClimb, leftDrop, leftClimb;


    @Override
    public void initialize() {
        rightDrop = new DigitalInput(ClimbConstants.kClimbSensorRightDownPort);
        rightClimb = new DigitalInput(ClimbConstants.kClimbSensorRightUpPort);
        leftDrop = new DigitalInput(ClimbConstants.kClimbSensorLeftDownPort);
        leftClimb = new DigitalInput(ClimbConstants.kClimbSensorLeftUpPort);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("rightDrop", rightDrop.get());
        SmartDashboard.putBoolean("rightClimb", rightClimb.get());
        SmartDashboard.putBoolean("leftDrop", leftDrop.get());
        SmartDashboard.putBoolean("leftClimb", leftClimb.get());

        SmartDashboard.putNumber("heartbeat", Timer.getFPGATimestamp());
    }

    @Override
    public void end(boolean interrupted) {
        rightDrop.close();
        rightClimb.close();
        leftDrop.close();
        leftClimb.close();
    }

}
