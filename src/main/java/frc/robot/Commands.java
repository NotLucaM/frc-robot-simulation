package frc.robot;

public class Commands {

    // vision
    private boolean visionWantedSnapshot = false;


    public synchronized boolean isVisionWantedSnapshot() {
        return visionWantedSnapshot;
    }

    public synchronized void setVisionWantedSnapshot(boolean visionWantedSnapshot) {
        this.visionWantedSnapshot = visionWantedSnapshot;
    }
}
