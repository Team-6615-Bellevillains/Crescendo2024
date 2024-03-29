package frc.robot.components.commands.climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.components.subsystems.climb.ClimbLeftSubsystem;
import frc.robot.utils.enums.Direction;

public class ClimbLeftCmd extends Command {

    private final ClimbLeftSubsystem climbLeftSubsystem;
    private final DigitalInput leftClimbSensor;
    private final Direction direction;

    public ClimbLeftCmd(ClimbLeftSubsystem climbLeftSubsystem, Direction direction) {
        this.climbLeftSubsystem = climbLeftSubsystem;
        this.direction = direction;

        leftClimbSensor = new DigitalInput(direction == Direction.UP ? ClimbConstants.kClimbSensorLeftUpPort : ClimbConstants.kClimbSensorLeftDownPort);

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
        return !leftClimbSensor.get();
    }
}
