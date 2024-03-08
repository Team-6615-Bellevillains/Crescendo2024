package frc.robot.utils;

// Used to determine the starting position of the robot during autonomous mode.
public enum Position {
    AMP, MIDDLE, SOURCE;

    @Override
    public String toString() {
        return switch (this) {
            case AMP -> "amp";
            case MIDDLE -> "middle";
            case SOURCE -> "source";
        };
    }
}
