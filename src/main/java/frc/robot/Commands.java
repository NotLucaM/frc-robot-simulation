package frc.robot;

import frc.subsystems.Vision;
import org.photonvision.common.hardware.VisionLEDMode;

public class Commands {

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
