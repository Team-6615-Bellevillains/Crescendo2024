// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbDownRightCmd;
import frc.robot.commands.ClimbDownLeftCmd;
import frc.robot.commands.ClimbUpLeftCmd;
import frc.robot.commands.ClimbUpRightCmd;
import frc.robot.commands.TestClimbSensorsCmd;
import frc.robot.commands.TestIntakeSensorCmd;
import frc.robot.commands.arm.IntakeRingCmd;
import frc.robot.commands.arm.IntakeRingUntilCapturedCmd;
// import frc.robot.commands.arm.RotateCmd;
import frc.robot.commands.arm.RotateToSpecificAngle;
import frc.robot.commands.arm.RotationControlJoystick;
import frc.robot.commands.arm.ShootCmd;
import frc.robot.commands.arm.ShootVoltageCmd;
import frc.robot.commands.arm.ShootVoltageFlywheel;
import frc.robot.commands.arm.TuneGravity;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.FieldOrientedDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.arm.RotationSubsystem;
import frc.robot.subsystems.arm.ShootingSubsystem;
import frc.robot.subsystems.arm.StorageSubsystem;
import frc.robot.subsystems.ClimbLeftSubsystem;
import frc.robot.subsystems.ClimbRightSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  private final StorageSubsystem storageSubsystem = new StorageSubsystem();
  private final ShootingSubsystem shootingSubsystem = new ShootingSubsystem();
  private final RotationSubsystem rotationSubsystem = new RotationSubsystem();
  private final ClimbLeftSubsystem climbLeftSubsystem = new ClimbLeftSubsystem();
  private final ClimbRightSubsystem climbRightSubsystem = new ClimbRightSubsystem();
  public Trigger lefTrigger;
  public Trigger rightTrigger;

  private boolean isHoldingSpeakerAngle = true;

  private final ShootVoltageFlywheel ampShooter = new ShootVoltageFlywheel(shootingSubsystem, -3.25, storageSubsystem, 4);  
  private final ShootVoltageFlywheel trapShooter = new ShootVoltageFlywheel(shootingSubsystem, -7, storageSubsystem, 6);
  private final ShootVoltageFlywheel speakerShooter = new ShootVoltageFlywheel(shootingSubsystem, -10, storageSubsystem, 10);

  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  CommandXboxController operatorXbox = new CommandXboxController(1);

  public static double controlMultiplier = 1.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();


    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));
                                                                                                

    FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive(drivebase, () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND) * controlMultiplier,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND) * controlMultiplier,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND)* controlMultiplier);

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? fieldOrientedDrive : driveFieldOrientedDirectAngleSim);

        NamedCommands.registerCommand("shoot", speakerShooter);

        NamedCommands.registerCommand(
          "rotateDown", Commands.runOnce(() -> isHoldingSpeakerAngle = !isHoldingSpeakerAngle)
      .andThen(new RotateToSpecificAngle(rotationSubsystem, () -> isHoldingSpeakerAngle)));

        NamedCommands.registerCommand("intake", new IntakeRingUntilCapturedCmd(storageSubsystem, shootingSubsystem)
      .andThen(Commands.runOnce(() -> isHoldingSpeakerAngle = !isHoldingSpeakerAngle))
      .andThen(new RotateToSpecificAngle(rotationSubsystem, () -> isHoldingSpeakerAngle)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverXbox, 2).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox,
    //                    2).whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
    operatorXbox.y().onTrue(new IntakeRingUntilCapturedCmd(storageSubsystem, shootingSubsystem)
      .andThen(Commands.runOnce(() -> isHoldingSpeakerAngle = !isHoldingSpeakerAngle))
      .andThen(new RotateToSpecificAngle(rotationSubsystem, () -> isHoldingSpeakerAngle)));
    operatorXbox.b().onTrue(trapShooter); // Stsgr Shooter
    operatorXbox.x().onTrue(speakerShooter); // Speaker Shooter
    // operatorXbox.rightBumper().onTrue(new RotateCmd(rotationSubsystem, 3, -0.1)); //Rotate to Amp
    // operatorXbox.leftBumper().onTrue(new RotateCmd(rotationSubsystem, 8, -0.1)); //Rotate to Speaker
    // operatorXbox.leftTrigger().onTrue(new RotateCmd(rotationSubsystem, 8, 0.1)); //Rotate to stage
    // operatorXbox.rightTrigger().onTrue(new RotateCmd(rotationSubsystem, 8, 0.1)); //Rotate to intake
    // operatorXbox.leftBumper().whileTrue(new TuneGravity(rotationSubsystem));
    // rotationSubsystem.setDefaultCommand(new RotationControlJoystick(rotationSubsystem, operatorXbox::getLeftY));

    operatorXbox.a().onTrue(
      Commands.runOnce(() -> isHoldingSpeakerAngle = !isHoldingSpeakerAngle)
      .andThen(new RotateToSpecificAngle(rotationSubsystem, () -> isHoldingSpeakerAngle))
    );

    //  new JoystickButton(driverXbox, 4).onTrue(Commands.parallel(new ClimbUpLeftCmd(climbLeftSubsystem),new ClimbUpRightCmd(climbRightSubsystem) ));
    //  new JoystickButton(driverXbox, 1).onTrue(Commands.parallel(new ClimbDownLeftCmd(climbLeftSubsystem),new ClimbDownRightCmd(climbRightSubsystem) ));

  // new JoystickButton(driverXbox, 1).onTrue(new ClimbDownLeftCmd(climbLeftSubsystem));
   //new JoystickButton(driverXbox, 4).onTrue(new ClimbUpLeftCmd(climbLeftSubsystem));

   operatorXbox.start().whileTrue(new TestIntakeSensorCmd());
    //    new JoystickButton.(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return speakerShooter.andThen(drivebase.getAutonomousCommand("drive to note", true));
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
