package frc.robot;

import frc.constants.DriveConstants;
import frc.util.Joystick;
import org.littletonrobotics.junction.Logger;

import static frc.util.Util.handleDeadBand;

public class OperatorInterface {

    private final Logger log = Logger.getInstance();
    private final Joystick driveStick = new Joystick(0),
            turnStick = new Joystick(1);

    public void update(Commands commands, RobotState state) {
        updateDriveCommands(commands, state);
    }

    private void updateDriveCommands(Commands commands, RobotState state) {
        commands.setDriveSlowTurnLeft(turnStick.getPOV(0) == 270);
        commands.setDriveSlowMoveBackwards(driveStick.getPOV(0) == 180);

        commands.setDriveTeleop(
                handleDeadBand(-driveStick.getY(), DriveConstants.DEAD_BAND), handleDeadBand(turnStick.getX(), DriveConstants.DEAD_BAND),
                turnStick.getTrigger(),
                turnStick.getPOV(0) == 90 || turnStick.getPOV(0) == 270,
                driveStick.getPOV(0) == 0 || driveStick.getPOV(0) == 180,
                driveStick.getTrigger());
    }
}
