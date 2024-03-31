package frc.robot.components.commands.arm.rotation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.components.subsystems.pivot.RotationSubsystem;

import java.util.function.DoubleSupplier;

public class ArmRotateUsingJoystick extends Command {

    private final RotationSubsystem rotationSubsystem;
    private final DoubleSupplier joystickInputSupplier;

    public ArmRotateUsingJoystick(RotationSubsystem rotationSubsystem, DoubleSupplier joystickInputSupplier) {
        this.rotationSubsystem = rotationSubsystem;
        this.joystickInputSupplier = joystickInputSupplier;

        this.addRequirements(rotationSubsystem);
    }

    @Override
    public void execute() {
        double suppliedInput = MathUtil.applyDeadband(joystickInputSupplier.getAsDouble(), 0.10);
        double desiredVelocityRadiansPerSec = suppliedInput * Units.degreesToRadians(150);

        // SmartDashboard.putNumber("DESIRED arm rotation velocity DESIRED", desiredVelocityRadiansPerSec);

        rotationSubsystem.setMotorVoltage(rotationSubsystem.calculateFeedforward(rotationSubsystem.getRotationEncoderPositionInRadians(), desiredVelocityRadiansPerSec));
    }

    @Override
    public void end(boolean interrupted) {
        rotationSubsystem.setMotorVoltage(0);
    }

}
