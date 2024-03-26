package frc.robot.components.superstructures;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.components.commands.climb.ClimbLeftNoMagnetCmd;
import frc.robot.components.commands.climb.ClimbRightNoMagnetCmd;
import frc.robot.components.commands.climb.ForceClimbLeftCmd;
import frc.robot.components.commands.climb.ForceClimbRightCmd;
import frc.robot.components.subsystems.climb.ClimbLeftSubsystem;
import frc.robot.components.subsystems.climb.ClimbRightSubsystem;
import frc.robot.utils.enums.Direction;

public class Climb {
    private final ClimbLeftSubsystem climbLeftSubsystem;
    private final ClimbRightSubsystem climbRightSubsystem;

    public Climb() {
        climbLeftSubsystem = new ClimbLeftSubsystem();
        climbRightSubsystem = new ClimbRightSubsystem();
    }

    public Command climbUpNoMagnet() {
        return new ClimbLeftNoMagnetCmd(climbLeftSubsystem, Direction.UP).alongWith(new ClimbRightNoMagnetCmd(climbRightSubsystem, Direction.UP));
    }

    public Command climbDownNoMagnet() {
        return new ClimbLeftNoMagnetCmd(climbLeftSubsystem, Direction.DOWN).onlyIf(() -> climbLeftSubsystem.getClimbRotations() >= Constants.ClimbConstants.CLIMB_LEFT_DOWN_THRESHOLD_ROTATIONS)
                .alongWith(new ClimbRightNoMagnetCmd(climbRightSubsystem, Direction.DOWN).onlyIf(() -> climbRightSubsystem.getClimbRotations() >= Constants.ClimbConstants.CLIMB_RIGHT_DOWN_THRESHOLD_ROTATIONS));
    }

    public Command forceClimbDownLeft() {
        return new ForceClimbLeftCmd(climbLeftSubsystem, Direction.DOWN);
    }

    public Command forceClimbDownRight() {
        return new ForceClimbRightCmd(climbRightSubsystem, Direction.DOWN);
    }

    public Command forceClimbUpLeft() {
        return new ForceClimbLeftCmd(climbLeftSubsystem, Direction.UP);
    }

    public Command forceClimbUpRight() {
        return new ForceClimbRightCmd(climbRightSubsystem, Direction.UP);
    }
}
