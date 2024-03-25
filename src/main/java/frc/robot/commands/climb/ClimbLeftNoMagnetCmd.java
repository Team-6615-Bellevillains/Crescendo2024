package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbLeftSubsystem;
import frc.robot.utils.enums.Direction;

public class ClimbLeftNoMagnetCmd extends Command {

    private final ClimbLeftSubsystem climbLeftSubsystem;
    private final Direction direction;

    public ClimbLeftNoMagnetCmd(ClimbLeftSubsystem climbLeftSubsystem, Direction direction) {
        this.climbLeftSubsystem = climbLeftSubsystem;
        this.direction = direction;

        addRequirements(climbLeftSubsystem);
    }

    @Override
    public void initialize() {
        climbLeftSubsystem.setClimbSpeedPercentage(direction == Direction.UP ? 1 : -1);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        climbLeftSubsystem.setClimbSpeedPercentage(0);
    }

    @Override
    public boolean isFinished() {
        return direction == Direction.UP ?
                climbLeftSubsystem.getClimbRotations() >= ClimbConstants.CLIMB_LEFT_UP_THRESHOLD_ROTATIONS
                : climbLeftSubsystem.getClimbRotations() <= ClimbConstants.CLIMB_LEFT_DOWN_THRESHOLD_ROTATIONS;
    }
}
