package frc.subsystems;

import frc.robot.Commands;
import frc.robot.State;
import frc.util.Util;

public abstract class SubsystemBase {

    private final String name;

    protected SubsystemBase() {
         name = Util.classToJsonName(getClass());
    }

    public abstract void update(Commands commands, State state);
    public abstract void write(State state);
    public abstract void read(State state);
    public abstract void configure();

    public void log() {}

    @Override
    public String toString() {
        return name;
    }
}
