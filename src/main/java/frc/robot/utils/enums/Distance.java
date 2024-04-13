package frc.robot.utils.enums;

// Used to determine the distance the robot should go for an action
public enum Distance {
    CLOSE, FAR;

    @Override
    public String toString() {
        return switch (this) {
            case CLOSE -> "close";
            case FAR -> "far";
        };
    }
}