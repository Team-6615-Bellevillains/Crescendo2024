package frc.robot;

// Global Robot State
public class State {
    private static State instance;

    private State() {
        instance = this;
    }

    public static State getInstance() {
        return instance;
    }

    public boolean isIntaking() {
        return intaking;
    }

    public void setIntakingState(boolean intaking) {
        this.intaking = intaking;
    }

    public boolean isComingUpWithNote() {
        return comingUpWithNote;
    }

    public void setComingUpWithNote(boolean rumbling) {
        this.comingUpWithNote = rumbling;
    }

    public boolean intaking;
    public boolean comingUpWithNote;
}
