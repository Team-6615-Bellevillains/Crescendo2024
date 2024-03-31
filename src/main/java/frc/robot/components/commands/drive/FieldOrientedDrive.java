package frc.robot.components.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.State;
import frc.robot.components.subsystems.drive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.DriveConstants;

public class FieldOrientedDrive extends Command {
    private static final double SQRT_2 = Math.sqrt(2);
    private static final boolean USING_NEW_SPEED_METHOD = true;

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, vTheta;
    private final BooleanSupplier angleLeft, angleCenter, angleRight, shouldSlowWhenIntakingSupplier;
    private final double maxAngularVelocity;
    private boolean shouldSlowWhenIntaking;


    public FieldOrientedDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vTheta, BooleanSupplier angleLeft, BooleanSupplier angleCenter, BooleanSupplier angleRight, BooleanSupplier shouldSlowWhenIntakingSupplier) {
        this.swerve = swerve;

        this.vX = vX;
        this.vY = vY;
        this.vTheta = vTheta;

        this.angleLeft = angleLeft;
        this.angleCenter = angleCenter;
        this.angleRight = angleRight;

        this.shouldSlowWhenIntakingSupplier = shouldSlowWhenIntakingSupplier;

        this.maxAngularVelocity = SwerveMath.calculateMaxAngularVelocity(
                swerve.maximumSpeed,
                Math.abs(swerve.getSwerveDriveConfiguration().moduleLocationsMeters[0].getX()),
                Math.abs(swerve.getSwerveDriveConfiguration().moduleLocationsMeters[0].getY()));

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Convert from blue alliance origin rotation to red alliance
        if (swerve.shouldFlipRotation && !swerve.rotationHasBeenFlipped) {
            swerve.resetOdometry(swerve.getPose().rotateBy(Rotation2d.fromDegrees(180)));
            swerve.rotationHasBeenFlipped = true;
        }

        swerve.setDriveHeadingCorrection(true);
        shouldSlowWhenIntaking = shouldSlowWhenIntakingSupplier.getAsBoolean();
    }


    private Rotation2d getShootingRotation() {
        if (angleLeft.getAsBoolean()) {
            return Constants.DriveConstants.LEFT_SHOOTER_ANGLE;
        }

        if (angleCenter.getAsBoolean()) {
            return Constants.DriveConstants.CENTER_SHOOTER_ANGLE;
        }

        if (angleRight.getAsBoolean()) {
            return Constants.DriveConstants.RIGHT_SHOOTER_ANGLE;
        }

        return null;
    }

    @Override
    public void execute() {
        double inputScalar = 1;

        if (shouldSlowWhenIntaking && State.getInstance().isIntaking()) {
            inputScalar = DriveConstants.SLOW_SPEED_SCALAR;
        }

        double xCubed = Math.pow(vX.getAsDouble(), 3) * inputScalar;
        double yCubed = Math.pow(vY.getAsDouble(), 3) * inputScalar;

        double xSpeed, ySpeed;
        if (USING_NEW_SPEED_METHOD) {
            double norm = Math.hypot(xCubed, yCubed);
            xSpeed = Math.signum(xCubed) * Math.min(Math.abs(xCubed*SQRT_2), norm) * swerve.maximumSpeed;
            ySpeed = Math.signum(yCubed) * Math.min(Math.abs(yCubed*SQRT_2), norm) * swerve.maximumSpeed;
        } else {
            xSpeed = xCubed * swerve.maximumSpeed;
            ySpeed = yCubed * swerve.maximumSpeed;
        }

        ChassisSpeeds desiredSpeeds;
        Rotation2d fixedShootingRotation = getShootingRotation();

        // no fixed angle button is pressed, use right joystick
        if (fixedShootingRotation == null) {
            double thetaCubed = Math.pow(vTheta.getAsDouble(), 3) * maxAngularVelocity * inputScalar;
            desiredSpeeds = swerve.getSwerveController().getRawTargetSpeeds(xSpeed, ySpeed, thetaCubed);
        } else {
            desiredSpeeds = swerve.getSwerveController().getRawTargetSpeeds(xSpeed, ySpeed, fixedShootingRotation.getRadians(), swerve.getHeading().getRadians());
        }

        // Limit velocity to prevent tipping
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                swerve.getSwerveDriveConfiguration());
        // SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        // SmartDashboard.putString("Translation", translation.toString());

        // Make the robot move
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.setDriveHeadingCorrection(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
