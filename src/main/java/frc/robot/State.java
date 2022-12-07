package frc.robot;

import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class State {

    public double visionLatency = 0.0;
    public Double visionDistanceToTarget = 0.0;
    public PhotonTrackedTarget visionTarget = null;
}
