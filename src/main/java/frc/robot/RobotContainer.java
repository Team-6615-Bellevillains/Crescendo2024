// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.Constants.ArmConstants.ShooterConstants;
import frc.robot.commands.ClimbLeftCmd;
import frc.robot.commands.ClimbRightCmd;
import frc.robot.commands.arm.ArmRotate;
import frc.robot.commands.arm.IntakeRingUntilCaptured;
import frc.robot.commands.arm.ShootCmd;
import frc.robot.commands.drive.FieldOrientedDrive;
import frc.robot.subsystems.ClimbLeftSubsystem;
import frc.robot.subsystems.ClimbRightSubsystem;
import frc.robot.subsystems.arm.RotationSubsystem;
import frc.robot.subsystems.arm.ShootingSubsystem;
import frc.robot.subsystems.arm.StorageSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utils.Direction;
import frc.robot.utils.Pathing;
import frc.robot.utils.Position;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));
    private final StorageSubsystem storageSubsystem = new StorageSubsystem();
    private final ShootingSubsystem shootingSubsystem = new ShootingSubsystem();
    private final RotationSubsystem rotationSubsystem = new RotationSubsystem();
    private final ClimbLeftSubsystem climbLeftSubsystem = new ClimbLeftSubsystem();
    private final ClimbRightSubsystem climbRightSubsystem = new ClimbRightSubsystem();

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

    private final SendableChooser<Position> autonStartingPositionChooser = new SendableChooser<>();
    private final SendableChooser<Pathing> autonPathingChooser = new SendableChooser<>();
    private final SendableChooser<Boolean> slowWhenIntakingChooser = new SendableChooser<>();

    private ShootCmd getTrapShooterInstance() {
        return new ShootCmd(shootingSubsystem, ShooterConstants.SHOOTING_TRAP_SHOOTER_VOLTAGE, storageSubsystem,
                ShooterConstants.STORAGE_TRAP_SHOOTER_VOLTAGE);
    }

    private ShootCmd getSpeakerShooterInstance() {
        return new ShootCmd(shootingSubsystem, ShooterConstants.SHOOTING_SPEAKER_SHOOTER_VOLTAGE, storageSubsystem,
                ShooterConstants.STORAGE_SPEAKER_SHOOTER_VOLTAGE);
    }

    public static Direction armHoldDirection = Direction.UP;
    public static double controlMultiplier = 1.0;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        autonStartingPositionChooser.addOption("Amp", Position.AMP);
        autonStartingPositionChooser.addOption("Middle", Position.MIDDLE);
        autonStartingPositionChooser.addOption("Source", Position.SOURCE);
        autonStartingPositionChooser.setDefaultOption("Amp", Position.AMP);
        SmartDashboard.putData("Starting Position", autonStartingPositionChooser);

        autonPathingChooser.addOption("Just Shoot", Pathing.DONT_MOVE);
        autonPathingChooser.addOption("Shoot and Back Up", Pathing.BACK_UP);
        autonPathingChooser.addOption("Two Note", Pathing.GO_FOR_SECOND_NOTE);
        autonPathingChooser.setDefaultOption("Shoot and Back Up", Pathing.BACK_UP);
        SmartDashboard.putData("Auton Pathing", autonPathingChooser);

        slowWhenIntakingChooser.addOption("Yes", true);
        slowWhenIntakingChooser.addOption("No", false);
        slowWhenIntakingChooser.setDefaultOption("No", false);
        SmartDashboard.putData("Slow When Intaking?", slowWhenIntakingChooser);

        // Configure the trigger bindings
        configureBindings();

        Command driveFieldOrientedDirectAngleSim = swerveSubsystem.simDriveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> driverXbox.getRawAxis(2));

        FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive(swerveSubsystem,
                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND) * controlMultiplier,
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND) * controlMultiplier,
                () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                        OperatorConstants.RIGHT_X_DEADBAND) * controlMultiplier);

        swerveSubsystem.setDefaultCommand(
                !RobotBase.isSimulation() ? fieldOrientedDrive : driveFieldOrientedDirectAngleSim);

        rotationSubsystem.activateArmHold();
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
        driverXbox.y().onTrue(Commands.parallel(new ClimbLeftCmd(climbLeftSubsystem, Direction.UP),
                new ClimbRightCmd(climbRightSubsystem, Direction.UP)));
        driverXbox.a().onTrue(Commands.parallel(new ClimbLeftCmd(climbLeftSubsystem, Direction.DOWN),
                new ClimbRightCmd(climbRightSubsystem, Direction.DOWN)));

        operatorXbox.b().onTrue(getTrapShooterInstance());
        operatorXbox.x().onTrue(getSpeakerShooterInstance());
        operatorXbox.y().onTrue(new ArmRotate(rotationSubsystem).andThen(new IntakeRingUntilCaptured(storageSubsystem, shootingSubsystem, () -> slowWhenIntakingChooser.getSelected())
                .andThen(new ArmRotate(rotationSubsystem))));
        operatorXbox.a().onTrue(new ArmRotate(rotationSubsystem));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        final Position startingPosition = autonStartingPositionChooser.getSelected();
        final Pathing autonPathing = autonPathingChooser.getSelected();

        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        commandGroup.addCommands(getSpeakerShooterInstance());

        return switch (autonPathing) {
            case DONT_MOVE -> commandGroup;
            case BACK_UP -> {
                commandGroup.addCommands(swerveSubsystem.getAutonomousCommand(startingPosition + " back up", true));
                yield commandGroup;
            }
            case GO_FOR_SECOND_NOTE -> {
                commandGroup.addCommands(
                        swerveSubsystem.getAutonomousCommand(startingPosition + " note", true),
                        new ArmRotate(rotationSubsystem),
                        Commands.parallel(
                                Commands.runOnce(() -> swerveSubsystem
                                        .driveFieldOriented(new ChassisSpeeds(AutonConstants.intakeForwardsSpeedMetersPerSecond, 0, 0)), swerveSubsystem),
                                new IntakeRingUntilCaptured(storageSubsystem, shootingSubsystem)),
                        new ArmRotate(rotationSubsystem),
                        swerveSubsystem.getAutonomousCommand(startingPosition + " note return", false),
                        getSpeakerShooterInstance(), 
                        swerveSubsystem.getAutonomousCommand(startingPosition + " back up", false));
                yield commandGroup;
            }
        };
    }

    public void setMotorBrake(boolean brake) {
        swerveSubsystem.setMotorBrake(brake);
    }
}
