package frc.util.control;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class Pigeon extends WPI_PigeonIMU {

    public Pigeon(int deviceNumber) {
        super(deviceNumber);
    }

    public Pigeon(TalonSRX talon) {
        super(talon);
    }
}
