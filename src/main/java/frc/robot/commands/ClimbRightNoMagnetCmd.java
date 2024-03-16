package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbRightSubsystem;
import frc.robot.utils.Direction;

public class ClimbRightNoMagnetCmd extends Command {

    private final ClimbRightSubsystem climbRightSubsystem;
    private final Direction direction;

    public ClimbRightNoMagnetCmd(ClimbRightSubsystem climbRightSubsystem, Direction direction) {
        this.climbRightSubsystem = climbRightSubsystem;
        this.direction = direction;

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
        return direction == Direction.UP ?
                climbRightSubsystem.getClimbRotations() >= ClimbConstants.CLIMB_RIGHT_UP_THRESHOLD_ROTATIONS
                : climbRightSubsystem.getClimbRotations() <= ClimbConstants.CLIMB_RIGHT_DOWN_THRESHOLD_ROTATIONS;
    }
}
