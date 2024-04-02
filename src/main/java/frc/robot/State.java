package frc.robot;

// Global Robot State
public class State {
    private static State instance;

    private State() {
        instance = this;
    }

    public static State getInstance() {
        if (instance == null) {
            new State();
        }

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

    public boolean intaking = false;
    public boolean comingUpWithNote = false;
}
