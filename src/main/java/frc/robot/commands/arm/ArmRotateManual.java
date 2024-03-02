package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.RotationSubsystem;

import static frc.robot.Constants.ArmConstants.RotationConstants;

public class ArmRotateManual extends Command {

    private final RotationSubsystem rotationSubsystem;
    private final DoubleSupplier joystickInputSupplier;

    public ArmRotateManual(RotationSubsystem rotationSubsystem, DoubleSupplier joystickInputSupplier) {
        this.rotationSubsystem = rotationSubsystem;
        this.joystickInputSupplier = joystickInputSupplier;

        this.addRequirements(rotationSubsystem);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Arm position degrees", rotationSubsystem.getRotationEncoderPositionInDegrees());
        if(rotationSubsystem.getRotationEncoderPositionInDegrees() < RotationConstants.SLOW_DRIVING_ANGLE_THRESHOLD_DEGREES)
        {
            RobotContainer.controlMultiplier = 0.6;
        }
        else
        {
            RobotContainer.controlMultiplier = 1;
        }
        double suppliedInput = MathUtil.applyDeadband(joystickInputSupplier.getAsDouble(), 0.10);
        double desiredVelocityRadiansPerSec = suppliedInput * Units.degreesToRadians(100);
        
        SmartDashboard.putNumber("DESIRED arm rotation velocity DESIRED", desiredVelocityRadiansPerSec);
        rotationSubsystem.setMotorVoltage(rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(), desiredVelocityRadiansPerSec));
    }

    @Override
    public void end(boolean interrupted) {
        rotationSubsystem.setMotorVoltage(rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(), 0));
    }
    
}
