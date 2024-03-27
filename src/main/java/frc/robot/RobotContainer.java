// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.components.commands.drive.FieldOrientedDrive;
import frc.robot.components.subsystems.drive.SwerveSubsystem;
import frc.robot.components.superstructures.Climb;
import frc.robot.components.superstructures.Pivot;
import frc.robot.utils.enums.Direction;
import frc.robot.utils.enums.FlipUtil;
import frc.robot.utils.enums.Pathing;
import frc.robot.utils.enums.Position;

import java.io.File;

import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Global robot state
    public static State state = new State();

    // Subsystem instances for the robot, representing the layer between our code
    // and direct motor control
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
    private final Climb climb = new Climb();
    private final Pivot pivot = new Pivot();

    // Xbox controllers for driver and operator with indices 0 and 1, indicating USB
    // order on the driver station
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

    // Dashboard buttons to select autonomous starting position, autonomous
    // strategy, and whether to slowdown when intaking on the dashboard
    private final SendableChooser<Position> autonStartingPositionChooser = new SendableChooser<>();
    private final SendableChooser<Pathing> autonPathingChooser = new SendableChooser<>();
    private final SendableChooser<Boolean> slowWhenIntakingChooser = new SendableChooser<>();

    private void rumbleControllers(double rumblePercent) {
        driverXbox.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumblePercent);
        operatorXbox.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumblePercent);
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Setup the dashboard to allow the user to select the starting position and auton pathing
        autonStartingPositionChooser.addOption("Amp", Position.AMP);
        autonStartingPositionChooser.addOption("Middle", Position.MIDDLE);
        autonStartingPositionChooser.addOption("Source", Position.SOURCE);
        autonStartingPositionChooser.setDefaultOption("Amp", Position.AMP);
        SmartDashboard.putData("Starting Position", autonStartingPositionChooser);

        autonPathingChooser.addOption("Just Shoot", Pathing.DONT_MOVE);
        autonPathingChooser.addOption("Shoot and Back Up", Pathing.BACK_UP);
        autonPathingChooser.addOption("Two Note", Pathing.GO_FOR_SECOND_NOTE);

        //Three note autonomous paths
        autonPathingChooser.addOption("Three Note", Pathing.GO_FOR_THIRD_NOTE);

        autonPathingChooser.setDefaultOption("Shoot and Back Up", Pathing.BACK_UP);
        SmartDashboard.putData("Auton Pathing", autonPathingChooser);

        slowWhenIntakingChooser.addOption("Yes", true);
        slowWhenIntakingChooser.addOption("No", false);
        slowWhenIntakingChooser.setDefaultOption("No", false);
        SmartDashboard.putData("Slow When Intaking?", slowWhenIntakingChooser);

        // Set up trigger bindings for different controller buttons
        // - Mapping the commands with various button presses on the Xbox controllers
        configureBindings();

        FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive(
                swerveSubsystem,
                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
                () -> driverXbox.leftTrigger().getAsBoolean(),
                () -> driverXbox.leftBumper().getAsBoolean() || driverXbox.rightBumper().getAsBoolean(),
                () -> driverXbox.rightTrigger().getAsBoolean(),
                slowWhenIntakingChooser::getSelected
        );

        // Make the field-oriented drive command always run
        swerveSubsystem.setDefaultCommand(fieldOrientedDrive);

        // Activate arm hold mechanism upon robot startup
        pivot.activateArmHold();

        new Trigger(() -> state.isRumbling())
            .onTrue(Commands.runOnce(() -> rumbleControllers(OperatorConstants.RUMBLE_POWER_PERCENTAGE)))
            .onFalse(Commands.runOnce(() -> rumbleControllers(0)));

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the
     * named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        driverXbox.b().onTrue((new InstantCommand(swerveSubsystem::zeroGyro)));

        driverXbox.y().onTrue(climb.climbUpNoMagnet());
        driverXbox.a().onTrue(climb.climbDownNoMagnet());

         driverXbox.leftBumper().and(driverXbox.start()).whileTrue(climb.forceClimbDownLeft());
         driverXbox.leftTrigger().and(driverXbox.start()).whileTrue(climb.forceClimbUpLeft());
         driverXbox.rightBumper().and(driverXbox.start()).whileTrue(climb.forceClimbDownRight());
         driverXbox.rightTrigger().and(driverXbox.start()).whileTrue(climb.forceClimbUpRight());

    
        operatorXbox.x().onTrue(pivot.speakerShooter());
        operatorXbox.a().whileTrue(pivot.intakeRingManual());
        operatorXbox.y().onTrue(pivot.intakeFromFloorThenReset());

        operatorXbox.start().onTrue(pivot.switchHoldDirectionAndHold());

        operatorXbox.leftBumper().whileTrue(pivot.spinUp());
        operatorXbox.leftTrigger().whileTrue(pivot.aimToSpeakerAndSpinUp()).onFalse(pivot.rotateArmAndHold(Direction.UP));
        operatorXbox.rightTrigger().onTrue(pivot.feedNote());
       
        operatorXbox.povDown().whileTrue(pivot.intakeFromSource()).onFalse(pivot.rotateArmAndHold(Direction.UP));
        operatorXbox.povLeft().whileTrue(pivot.intakeFromSource()).onFalse(pivot.rotateArmAndHold(Direction.UP));
    }

    // Returns the Command to run during the autonomous phase
    // Builds a SequentialCommandGroup based on the selected autonomous strategy
    public Command getAutonomousCommand() {
        final Position startingPosition = autonStartingPositionChooser.getSelected();
        final Pathing autonPathing = autonPathingChooser.getSelected();

        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        commandGroup.addCommands(Commands.runOnce(() -> {
            swerveSubsystem.autonRan = true;
            swerveSubsystem.shouldFlipRotation = FlipUtil.shouldFlipPath();
        }, swerveSubsystem));

        commandGroup.addCommands(pivot.speakerShooter());

        final double intakeForwardsSign = FlipUtil.shouldFlipPath() ? -1 : 1;

        // Determine the flow of autonomous commands based on the selected Pathing
        // option
        return switch (autonPathing) {
            // If "Don't Move" is selected, just run the shooter which is already setup
            // above
            case DONT_MOVE -> {
                commandGroup.addCommands(swerveSubsystem.resetOdometryToStartingPose(PathPlannerPath.fromPathFile(startingPosition + " back up")));
                yield commandGroup;
            }
            // If "Back Up" is selected, a command to back up is added to the command group
            // The second argument is true because we want to set the odometry to the position the bot starts in.
            case BACK_UP -> {
                commandGroup.addCommands(
                        Commands.waitSeconds(AutonConstants.BACKUP_WAIT_SECONDS), // allow teammates to pick up notes if necessary
                        swerveSubsystem.getAutonomousCommand(startingPosition + " back up", true)
                );
                yield commandGroup;
            }
            // If "Go for Second Note" is selected, a series of commands are added to get a
            // second note and shoot it
            case GO_FOR_SECOND_NOTE -> {
                commandGroup.addCommands(
                        // Move to the note based on the starting position.
                        // The second argument is true because we want to set the odometry to the position the bot starts in.
                        swerveSubsystem.getAutonomousCommand(startingPosition + " note", true),
                        // Rotate the arm into intake position
                        pivot.switchHoldDirectionAndHold(),
                        // Drive forwards at a slow speed while running the intake motors.
                        // Stop once the note has been obtained or AutonConstants.INTAKE_TIMEOUT_SECONDS seconds have passed, whichever comes first
                        Commands.parallel(
                                Commands.runOnce(() -> swerveSubsystem.driveFieldOriented(new ChassisSpeeds(intakeForwardsSign * AutonConstants.intakeForwardsSpeedMetersPerSecond, 0, 0)), swerveSubsystem),
                                pivot.intakeRingUntilCaptured().withTimeout(AutonConstants.INTAKE_TIMEOUT_SECONDS)
                        ),
                        // Rotate arm back into shooting position
                        pivot.switchHoldDirectionAndHold(),
                        // Move back to the speaker.
                        // The second argument is false because we don't want to keep our current odometry.
                        swerveSubsystem.getAutonomousCommand(startingPosition + " note return", false),
                        // Shoot the note!
                        pivot.speakerShooter(),
                        // go back across the line!
                        swerveSubsystem.getAutonomousCommand(startingPosition + " back up", false));
                yield commandGroup;
            }

            //If "Go for Third Note" is selected, a series of commands are added to get a
            // third note and shoot it

            case GO_FOR_THIRD_NOTE -> {
                commandGroup.addCommands(
                        // Move to the note based on the starting position.
                        // The second argument is true because we want to set the odometry to the position the bot starts in.
                        swerveSubsystem.getAutonomousCommand("Second note", true),
                        // Rotate the arm into intake position
                        // Drive forwards at a slow speed while running the intake motors.
                        // Stop once the note has been obtained or AutonConstants.INTAKE_TIMEOUT_SECONDS seconds have passed, whichever comes first
                        Commands.parallel(
                                Commands.runOnce(() -> swerveSubsystem.driveFieldOriented(new ChassisSpeeds(intakeForwardsSign * AutonConstants.intakeForwardsSpeedMetersPerSecond, 0, 0)), swerveSubsystem),
                                pivot.intakeRingUntilCaptured().withTimeout(AutonConstants.INTAKE_TIMEOUT_SECONDS)
                        ),
                        // Rotate arm back into shooting position
                        // Move back to the speaker.
                        // The second argument is false because we don't want to keep our current odometry.
                        Commands.parallel(
                            Commands.waitSeconds(0.3).andThen(swerveSubsystem.getAutonomousCommand("Second note return", false)),
                            pivot.switchHoldDirectionAndHold().withTimeout(2)
                        ),
                        
                        // Shoot the note!
                        pivot.speakerShooter(),

                        //Third note
                        swerveSubsystem.getAutonomousCommand("Third piece start", false),
                        Commands.parallel(
                                Commands.runOnce(() -> swerveSubsystem.driveFieldOriented(new ChassisSpeeds(intakeForwardsSign * Units.inchesToMeters(25.28), Units.inchesToMeters(25.28), 0)), swerveSubsystem),
                                pivot.intakeRingUntilCaptured().withTimeout(AutonConstants.INTAKE_TIMEOUT_SECONDS)
                        ),
                        swerveSubsystem.getAutonomousCommand("Third piece return", false),
                        pivot.speakerShooter(),
                        // go back across the line!
                        swerveSubsystem.getAutonomousCommand(startingPosition + " back up", false));
                yield commandGroup;
                
            }
        };
    }

    public void setMotorBrake(boolean brake) {
        swerveSubsystem.setMotorBrake(brake);
    }
}
