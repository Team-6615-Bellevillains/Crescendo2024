package frc.robot.components.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.components.subsystems.climb.ClimbRightSubsystem;
import frc.robot.utils.enums.Direction;

public class ForceClimbRightCmd extends Command {

    private final ClimbRightSubsystem climbRightSubsystem;
    private final Direction direction;

    public ForceClimbRightCmd(ClimbRightSubsystem climbRightSubsystem, Direction direction) {
        this.climbRightSubsystem = climbRightSubsystem;
        this.direction = direction;

        addRequirements(climbRightSubsystem);
    }

    @Override
    public void initialize() {
        climbRightSubsystem.setClimbSpeedPercentage(direction == Direction.UP ? 0.05 : -0.05);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        climbRightSubsystem.setClimbSpeedPercentage(0);
    }
}
