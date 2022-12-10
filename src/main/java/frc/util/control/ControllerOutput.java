package frc.util.control;

public class ControllerOutput {

    public enum Mode {
        PERCENT_OUTPUT, POSITION, VELOCITY, PROFILED_POSITION, PROFILED_VELOCITY
    }

    private Mode mode;
    private double reference, feedForward;

    public ControllerOutput() {
        this(Mode.PERCENT_OUTPUT);
    }

    public ControllerOutput(Mode controlMode) {
        this.mode = controlMode;
    }

    public void setTargetVelocity(double targetVelocity) {
        setTargetVelocity(targetVelocity, 0.0);
    }

    public void setTargetVelocity(double targetVelocity, double feedForward) {
        mode = Mode.VELOCITY;
        reference = targetVelocity;
        feedForward = feedForward;
    }

    public void setTargetPosition(double positionSetPoint) {
        setTargetPosition(positionSetPoint, 0.0);
    }

    public void setTargetPosition(double positionSetPoint, double feedForward) {
        mode = Mode.POSITION;
        reference = positionSetPoint;
        feedForward = feedForward;
    }

    public void setIdle() {
        setPercentOutput(0.0);
    }

    public void setPercentOutput(double output) {
        mode = Mode.PERCENT_OUTPUT;
        reference = output;
        feedForward = 0.0;
    }

    public double getReference() {
        return reference;
    }

    public double getArbitraryDemand() {
        return feedForward;
    }

    public Mode getControlMode() {
        return mode;
    }
}
