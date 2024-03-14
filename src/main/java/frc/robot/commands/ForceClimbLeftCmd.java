package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbLeftSubsystem;
import frc.robot.utils.Direction;

public class ForceClimbLeftCmd extends Command {

    private final ClimbLeftSubsystem climbLeftSubsystem;
    private final Direction direction;

    public ForceClimbLeftCmd(ClimbLeftSubsystem climbLeftSubsystem, Direction direction) {
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
}
