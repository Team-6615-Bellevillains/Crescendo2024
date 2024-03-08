package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.util.List;
import java.util.function.DoubleSupplier;

public class FieldOrientedDrive extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, vTheta;
    private final double maxAngularVelocity;

    public FieldOrientedDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vTheta) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.vTheta = vTheta;
        this.maxAngularVelocity = SwerveMath.calculateMaxAngularVelocity(
                swerve.maximumSpeed,
                Math.abs(swerve.getSwerveDriveConfiguration().moduleLocationsMeters[0].getX()),
                Math.abs(swerve.getSwerveDriveConfiguration().moduleLocationsMeters[0].getY()));

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setDriveHeadingCorrection(true);
    }

    @Override
    public void execute() {
        // Convert from blue alliance origin rotation to red alliance
        if (swerve.autonRan && swerve.shouldFlipRotation && !swerve.rotationHasBeenFlipped) {
            swerve.resetOdometry(swerve.getPose().rotateBy(Rotation2d.fromDegrees(180)));
            swerve.rotationHasBeenFlipped = true;
        }

        double xCubed = Math.pow(vX.getAsDouble(), 3) * swerve.maximumSpeed;
        double yCubed = Math.pow(vY.getAsDouble(), 3) * swerve.maximumSpeed;
        double thetaCubed = Math.pow(vTheta.getAsDouble(), 3) * maxAngularVelocity;

        ChassisSpeeds desiredSpeeds = swerve.getSwerveController().getRawTargetSpeeds(xCubed, yCubed, thetaCubed);

        // Limit velocity to prevent tippy
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

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
