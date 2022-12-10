package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.subsystems.Drive;
import frc.subsystems.Vision;
import frc.util.DriveOutputs;
import org.photonvision.common.hardware.VisionLEDMode;

public class Commands {

    /* Drive Commands */
    private Drive.State driveWantedState = Drive.State.NEUTRAL;
    // Teleop
    private double driveWantedThrottle, driveWantedWheel;
    private boolean driveWantsQuickTurn,
            driveWantsSlowTurn,
            driveWantedSlowTurnLeft,
            driveWantsBrake,
            driveWantsSlowMove,
            driveWantedSlowMoveBackwards;
    // Signal
    private DriveOutputs driveWantedSignal;
    // Path Following
    private Trajectory driveWantedTrajectory;
    public Pose2d driveWantedOdometryPose;
    // Turning
    private double driveWantedYawDegrees;

    public void setDriveOutputs(DriveOutputs outputs) {
        driveWantedState = Drive.State.OUTPUTS;
        driveWantedSignal = outputs;
    }

    public void setDriveFollowPath(Trajectory trajectory) {
        driveWantedState = Drive.State.FOLLOW_PATH;
        driveWantedTrajectory = trajectory;
    }

    public void setDriveTeleop() {
        setDriveTeleop(0.0, 0.0, false, false, false, false);
    }

    public void setDriveTeleop(double throttle, double wheel, boolean wantsQuickTurn, boolean wantsSlowTurn, boolean wantsSlowMove, boolean wantsBrake) {
        driveWantedState = Drive.State.TELEOP;
        driveWantedThrottle = throttle;
        driveWantedWheel = wheel;
        driveWantsQuickTurn = wantsQuickTurn;
        driveWantsSlowTurn = wantsSlowTurn;
        driveWantsSlowMove = wantsSlowMove;
        driveWantsBrake = wantsBrake;
    }

    public void setDriveNeutral() {
        driveWantedState = Drive.State.NEUTRAL;
    }

    public void setDriveYaw(double yawDegrees) {
        driveWantedState = Drive.State.TURN;
        driveWantedYawDegrees = yawDegrees;
    }

    public void setDriveSlowTurnLeft(boolean wantsSlowTurnLeft) {
        driveWantedSlowTurnLeft = wantsSlowTurnLeft;
    }

    public void setDriveSlowMoveBackwards(boolean wantsSlowMoveBack) {
        driveWantedSlowMoveBackwards = wantsSlowMoveBack;
    }

    public Drive.State getDriveWantedState() {
        return driveWantedState;
    }

    public boolean getDriveWantsQuickTurn() {
        return driveWantsQuickTurn;
    }

    public boolean getDriveWantsSlowTurn() {
        return driveWantsSlowTurn;
    }

    public boolean getDriveWantedSlowTurnLeft() {
        return driveWantedSlowTurnLeft;
    }

    public boolean getDriveWantsSlowMove() {
        return driveWantsSlowMove;
    }

    public boolean getDriveWantedSlowMoveBackwards() {
        return driveWantedSlowMoveBackwards;
    }

    public double getDriveWantedThrottle() {
        return driveWantedThrottle;
    }

    public double getDriveWantedWheel() {
        return driveWantedWheel;
    }

    public boolean getDriveWantsBreak() {
        return driveWantsBrake;
    }

    // vision
    public boolean visionWantedSnapshot = false;
    public Vision.State visionWantedLEDs = Vision.State.DRIVE;

    @Override
    protected Object clone() {
        Commands clone = new Commands();

        clone.visionWantedSnapshot = false;

        return clone;
    }
}
