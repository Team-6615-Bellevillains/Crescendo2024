package frc.robot;

public class State {
    public State() {
        setIntakingState(false);
    }

    public boolean isIntaking() {
        return intaking;
    }

    public void setIntakingState(boolean intaking) {
        this.intaking = intaking;
    }

    public boolean isRumbling() {
        return rumbling;
    }

    public void setRumbling(boolean rumbling) {
        this.rumbling = rumbling;
    }

    public boolean intaking;
    public boolean rumbling;
}
