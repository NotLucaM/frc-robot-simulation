package frc.robot;

import org.photonvision.common.hardware.VisionLEDMode;

public class Commands {

    // vision
    public boolean visionWantedSnapshot = false;
    public VisionLEDMode visionWantedLEDs = VisionLEDMode.kOff;

    @Override
    protected Object clone() {
        Commands clone = new Commands();

        clone.visionWantedSnapshot = false;

        return clone;
    }
}
