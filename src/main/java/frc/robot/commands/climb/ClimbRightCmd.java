package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbRightSubsystem;
import frc.robot.utils.enums.Direction;

public class ClimbRightCmd extends Command {

    private final ClimbRightSubsystem climbRightSubsystem;
    private final DigitalInput rightClimbSensor;
    private final Direction direction;

    public ClimbRightCmd(ClimbRightSubsystem climbRightSubsystem, Direction direction) {
        this.climbRightSubsystem = climbRightSubsystem;
        this.direction = direction;

        rightClimbSensor = new DigitalInput(direction == Direction.UP ? ClimbConstants.kClimbSensorRightUpPort : ClimbConstants.kClimbSensorRightDownPort);

        addRequirements(climbRightSubsystem);
    }

    @Override
    public void initialize() {
        climbRightSubsystem.setClimbSpeedPercentage(direction == Direction.UP ? 1 : -1);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        climbRightSubsystem.setClimbSpeedPercentage(0);
    }

    @Override
    public boolean isFinished() {
        return !rightClimbSensor.get();
    }
}
