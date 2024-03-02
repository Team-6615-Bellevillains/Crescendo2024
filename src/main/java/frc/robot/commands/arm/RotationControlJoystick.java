package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.RotationSubsystem;

public class RotationControlJoystick extends Command {

    private final RotationSubsystem rotationSubsystem;
    private final DoubleSupplier joystickInputSupplier;

    public RotationControlJoystick(RotationSubsystem rotationSubsystem, DoubleSupplier joystickInputSupplier) {
        this.rotationSubsystem = rotationSubsystem;
        this.joystickInputSupplier = joystickInputSupplier;

        this.addRequirements(rotationSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double suppliedInput = MathUtil.applyDeadband(joystickInputSupplier.getAsDouble(), 0.10);
        double desiredVelocity = suppliedInput * 20;
        
        SmartDashboard.putNumber("DESIRED arm rotation velocity DESIRED", desiredVelocity);
        rotationSubsystem.setMotorVoltage(rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(), Units.degreesToRadians(desiredVelocity)));
    }

    @Override
    public void end(boolean interrupted) {
        rotationSubsystem.setMotorVoltage(rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(), 0));
    }
    
}
