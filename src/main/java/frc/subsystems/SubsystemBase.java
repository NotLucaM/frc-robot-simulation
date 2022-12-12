package frc.subsystems;

import frc.robot.Commands;
import frc.robot.RobotState;
import frc.util.Util;
import org.littletonrobotics.junction.Logger;

public abstract class SubsystemBase {

    private final String name;
    protected final Logger log = Logger.getInstance();

    protected SubsystemBase() {
         name = Util.classToJsonName(getClass());
    }

    public abstract void update(Commands commands, RobotState state);
    public abstract void write();
    public abstract void read(RobotState state);
    public abstract void configure();

    public void simulate(RobotState state) {}
    public void log() {}

    @Override
    public String toString() {
        return name;
    }
}
