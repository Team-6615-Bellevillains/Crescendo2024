package frc.robot.components.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.State;
import frc.robot.components.subsystems.drive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.DriveConstants;

public class FieldOrientedDrive extends Command {
    private static final boolean USING_NEW_SPEED_CURVE = false;
    private static final InterpolatingDoubleTreeMap POWER_LERP = new InterpolatingDoubleTreeMap();

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


        POWER_LERP.clear();
        POWER_LERP.put(0.0, 0.0);
        POWER_LERP.put(0.15, 0.4 * 0.15); // scale the first 15% of power input down 60%
        POWER_LERP.put(1.0, 1.0);

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
            return DriveConstants.LEFT_SHOOTER_ANGLE;
        }

        if (angleCenter.getAsBoolean()) {
            return DriveConstants.CENTER_SHOOTER_ANGLE;
        }

        if (angleRight.getAsBoolean()) {
            return DriveConstants.RIGHT_SHOOTER_ANGLE;
        }

        return null;
    }

    @Override
    public void execute() {
        double velocityScalar = shouldSlowWhenIntaking && State.getInstance().isIntaking() ? DriveConstants.SLOW_SPEED_SCALAR : 1;

        double xInput = vX.getAsDouble();
        double yInput = vY.getAsDouble();

        Translation2d inputTranslation = new Translation2d(xInput, yInput);
        // clamp between [0, 1] to prevent any weirdness
        double magnitude = Math.max(0.0, Math.min(inputTranslation.getNorm(), 1.0));
        Rotation2d angle = inputTranslation.getAngle();
        double curvedMagnitude = USING_NEW_SPEED_CURVE ? POWER_LERP.get(magnitude) : Math.pow(magnitude, 3);

        double xVelocity = curvedMagnitude * angle.getCos() * swerve.maximumSpeed * velocityScalar;
        double yVelocity = curvedMagnitude * angle.getSin() * swerve.maximumSpeed * velocityScalar;

        ChassisSpeeds desiredSpeeds;
        Rotation2d fixedShootingRotation = getShootingRotation();

        // no fixed angle button is pressed, use right joystick
        if (fixedShootingRotation == null) {
            double thetaCubed = Math.pow(vTheta.getAsDouble(), 3) * maxAngularVelocity * velocityScalar;
            desiredSpeeds = swerve.getSwerveController().getRawTargetSpeeds(xVelocity, yVelocity, thetaCubed);
        } else {
            desiredSpeeds = swerve.getSwerveController().getRawTargetSpeeds(xVelocity, yVelocity, fixedShootingRotation.getRadians(), swerve.getHeading().getRadians());
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
