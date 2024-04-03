package frc.robot.components.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.components.subsystems.drive.SwerveSubsystem;

public class WheelRadiusCharacterization extends Command {
    private static final double driveRadius = Math.hypot(11.385, 11.385);
  private final DoubleSupplier gyroYawRadsSupplier;

  private final SwerveSubsystem swerveSubsystem;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(0.25);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.gyroYawRadsSupplier = () -> swerveSubsystem.getPose().getRotation().getRadians();

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = swerveSubsystem.getWheelRadiusCharacterizationPosition();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    swerveSubsystem.drive(new Translation2d(), omegaLimiter.calculate(1) * Units.degreesToRadians(360 * 1.5), false);

    // Get yaw and wheel positions
    double newRads = gyroYawRadsSupplier.getAsDouble();
    // SmartDashboard.putNumber("new Rads", newRads);
    accumGyroYawRads += MathUtil.angleModulus(newRads - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = swerveSubsystem.getWheelRadiusCharacterizationPosition();

    // SmartDashboard.putNumberArray("Positions", wheelPositiions);
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds());
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + currentEffectiveWheelRadius
              + " inches");
    }
  }
}
