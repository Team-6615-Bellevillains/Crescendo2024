package frc.robot.utils;

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
