package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.RotationSubsystem;
import frc.robot.utils.TunableProfiledPIDController;

public class TuneGravity extends Command {

    private final RotationSubsystem rotationSubsystem;

    public TuneGravity(RotationSubsystem rotationSubsystem) {
        this.rotationSubsystem = rotationSubsystem;
    }

    @Override
    public void initialize() {
        rotationSubsystem.setMotorVoltage(0);
        
        this.addRequirements(rotationSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Rotation Setpoint heartbeat", Timer.getFPGATimestamp());

        double ffOut = rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(),
                0);
        // double commandedVoltage = pidOut + ffOut;
        double commandedVoltage = ffOut;

        SmartDashboard.putNumber("Rotation ffout", ffOut);
        SmartDashboard.putNumber("Rotation commanded voltage", commandedVoltage);

        rotationSubsystem.setMotorVoltage(commandedVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Rotation position at end", rotationSubsystem.getRotationEncoderPositionInDegrees());
        SmartDashboard.putNumber("Rotation position at end", rotationSubsystem.getRotationEncoderPositionInDegrees());
        SmartDashboard.putNumber("Rotation velocity at end",
                rotationSubsystem.getRotationEncoderVelocityInDegreesPerSec());
        SmartDashboard.putNumber("Rotation heartbeat at end", Timer.getFPGATimestamp());

        // rotationSubsystem.setMotorVoltage(
        //         rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(), 0));
        rotationSubsystem.setMotorVoltage(0);
    }

}
