package frc.util;

public class Joystick extends edu.wpi.first.wpilibj.Joystick {

	public Joystick(int port) {
		super(port);
	}

	public String getName() {
		return "Joystick " + getPort();
	}
}
