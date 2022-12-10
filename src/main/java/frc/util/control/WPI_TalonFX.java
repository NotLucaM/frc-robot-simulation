/**
 * WPI Compliant motor controller class.
 * WPILIB's object model requires many interfaces to be implemented to use
 * the various features.
 * This includes...
 * - Software PID loops running in the robot controller
 * - LiveWindow/Test mode features
 * - Motor Safety (auto-turn off of motor if Set stops getting called)
 * - Single Parameter set that assumes a simple motor controller.
 */
package frc.util.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.WPI_MotorSafetyImplem;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.platform.DeviceType;
import com.ctre.phoenix.platform.PlatformJNI;
import com.ctre.phoenix.Logger;
import com.ctre.phoenix.WPI_CallbackHelper;
import com.ctre.phoenix.ErrorCode;

import java.util.ArrayList;

import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.hal.simulation.SimulatorJNI;
import edu.wpi.first.hal.HAL.SimPeriodicBeforeCallback;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimBoolean;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;

class WPI_AutoFeedEnable {
	private static WPI_AutoFeedEnable autoFeedEnable = new WPI_AutoFeedEnable();
	private SimPeriodicBeforeCallback simPeriodicCallback;

	public static WPI_AutoFeedEnable getInstance() {
		return autoFeedEnable;
	}

	private WPI_AutoFeedEnable() {
		simPeriodicCallback = HAL.registerSimPeriodicBeforeCallback(() -> {
			if (LoggedDriverStation.getInstance().getDSData().enabled) {
				Unmanaged.feedEnable(100);
			}
		});
	}
}

/**
 * CTRE Talon FX Motor Controller when used on CAN Bus.
 */
public class WPI_TalonFX extends TalonFX implements MotorController, Sendable, AutoCloseable {

	private String _description;
	private double _speed;

	private SimDevice m_simMotor;
	private SimDouble m_simPercOut;
	private SimDouble m_simMotorOutputLeadVoltage;
	private SimDouble m_simSupplyCurrent;
	private SimDouble m_simMotorCurrent;
	private SimDouble m_simVbat;

	private SimDevice m_simIntegSens;
	private SimDouble m_simIntegSensPos;
	private SimDouble m_simIntegSensAbsPos;
	private SimDouble m_simIntegSensRawPos;
	private SimDouble m_simIntegSensVel;

	private SimDevice m_simFwdLim;
	private SimBoolean m_simFwdLimInit;
	private SimBoolean m_simFwdLimInput;
	private SimBoolean m_simFwdLimValue;

	private SimDevice m_simRevLim;
	private SimBoolean m_simRevLimInit;
	private SimBoolean m_simRevLimInput;
	private SimBoolean m_simRevLimValue;

	// callbacks to register
	private SimValueCallback onValueChangedCallback = new OnValueChangedCallback();
	private Runnable onPeriodicCallback = new OnPeriodicCallback();

	// returned registered callbacks
	private ArrayList<CallbackStore> simValueChangedCallbacks = new ArrayList<CallbackStore>();
	private SimPeriodicBeforeCallback simPeriodicCallback;

	/**
	 * The default motor safety timeout IF calling application
	 * enables the feature.
	 */
	public static final double kDefaultSafetyExpiration = 0.1;

	/**
	 * Late-constructed motor safety, which ensures feature is off unless calling
	 * applications explicitly enables it.
	 */
	private WPI_MotorSafetyImplem _motorSafety = null;
	private final Object _lockMotorSaf = new Object();
	private double _motSafeExpiration = kDefaultSafetyExpiration;

	/**
	 * Constructor for motor controller
	 * @param deviceNumber device ID of motor controller
	 * @param canbus Name of the CANbus; can be a CANivore device name or serial number.
	 *               Pass in nothing or "rio" to use the roboRIO.
	 */
	public WPI_TalonFX(int deviceNumber, String canbus) {
		super(deviceNumber, canbus);
		_description = "Talon FX " + deviceNumber;

		SendableRegistry.addLW(this, "Talon FX ", deviceNumber);

		m_simMotor = SimDevice.create("CANMotor:Talon FX", deviceNumber);
		if(m_simMotor != null){
			WPI_AutoFeedEnable.getInstance();
			simPeriodicCallback = HAL.registerSimPeriodicBeforeCallback(onPeriodicCallback);

			m_simPercOut = m_simMotor.createDouble("percentOutput", Direction.kOutput, 0);
			m_simMotorOutputLeadVoltage = m_simMotor.createDouble("motorOutputLeadVoltage", Direction.kOutput, 0);

			m_simSupplyCurrent = m_simMotor.createDouble("supplyCurrent", Direction.kInput, 0);
			m_simMotorCurrent = m_simMotor.createDouble("motorCurrent", Direction.kInput, 0);
			m_simVbat = m_simMotor.createDouble("busVoltage", Direction.kInput, 12.0);

			SimDeviceSim sim = new SimDeviceSim("CANMotor:Talon FX");
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simSupplyCurrent, onValueChangedCallback, true));
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simMotorCurrent, onValueChangedCallback, true));
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simVbat, onValueChangedCallback, true));
		}

		String base = "Talon FX[" + deviceNumber + "]/";
		m_simIntegSens = SimDevice.create("CANEncoder:" + base + "Integrated Sensor");
		if(m_simIntegSens != null){
			m_simIntegSensPos = m_simIntegSens.createDouble("position", Direction.kOutput, 0);
			m_simIntegSensAbsPos = m_simIntegSens.createDouble("absolutePosition", Direction.kOutput, 0);

			m_simIntegSensRawPos = m_simIntegSens.createDouble("rawPositionInput", Direction.kInput, 0);
			m_simIntegSensVel = m_simIntegSens.createDouble("velocity", Direction.kInput, 0);

			SimDeviceSim sim = new SimDeviceSim("CANEncoder:" + base + "Integrated Sensor");
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simIntegSensRawPos, onValueChangedCallback, true));
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simIntegSensVel, onValueChangedCallback, true));
		}

		m_simFwdLim = SimDevice.create("CANDIO:" + base + "Fwd Limit");
		if(m_simFwdLim != null){
			m_simFwdLimInit = m_simFwdLim.createBoolean("init", Direction.kOutput, true);
			m_simFwdLimInput = m_simFwdLim.createBoolean("input", Direction.kOutput, true);

			m_simFwdLimValue = m_simFwdLim.createBoolean("value", Direction.kBidir, false);

			SimDeviceSim sim = new SimDeviceSim("CANDIO:" + base + "Fwd Limit");
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simFwdLimValue, onValueChangedCallback, true));
		}

		m_simRevLim = SimDevice.create("CANDIO:" + base + "Rev Limit");
		if(m_simRevLim != null){
			m_simRevLimInit = m_simRevLim.createBoolean("init", Direction.kOutput, true);
			m_simRevLimInput = m_simRevLim.createBoolean("input", Direction.kOutput, true);

			m_simRevLimValue = m_simRevLim.createBoolean("value", Direction.kBidir, false);

			SimDeviceSim sim = new SimDeviceSim("CANDIO:" + base + "Rev Limit");
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simRevLimValue, onValueChangedCallback, true));
		}
	}

	/**
	 * Constructor for motor controller
	 * @param deviceNumber device ID of motor controller
	 */
	public WPI_TalonFX(int deviceNumber) {
		this(deviceNumber, "");
	}

    // ----- Auto-Closable ----- //
    @Override
    public void close(){
        SendableRegistry.remove(this);
        if(m_simMotor != null) {
            m_simMotor.close();
            m_simMotor = null;
        }
        if(m_simIntegSens != null) {
            m_simIntegSens.close();
            m_simIntegSens = null;
        }
        if(m_simFwdLim != null) {
            m_simFwdLim.close();
            m_simFwdLim = null;
        }
        if(m_simRevLim != null) {
            m_simRevLim.close();
            m_simRevLim = null;
        }
        super.DestroyObject(); //Destroy the device
    }

	// ----- Callbacks for Sim ----- //
	private class OnValueChangedCallback implements SimValueCallback {
		@Override
		public void callback(String name, int handle, int direction, HALValue value) {
			String deviceName = SimDeviceDataJNI.getSimDeviceName(SimDeviceDataJNI.getSimValueDeviceHandle(handle));
			String physType = deviceName + ":" + name;
			PlatformJNI.JNI_SimSetPhysicsInput(DeviceType.TalonFX.value, getDeviceID(),
								               physType, WPI_CallbackHelper.getRawValue(value));
		}
	}

	private class OnPeriodicCallback implements Runnable {
		@Override
		public void run() {
			double value = 0;
			int err = 0;

			int deviceID = getDeviceID();

			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "PercentOutput");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simPercOut.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "MotorOutputLeadVoltage");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simMotorOutputLeadVoltage.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "BusVoltage");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simVbat.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "CurrentSupply");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simSupplyCurrent.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "CurrentStator");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simMotorCurrent.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "IntegSensPos");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simIntegSensPos.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "IntegSensAbsPos");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simIntegSensAbsPos.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "IntegSensRawPos");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simIntegSensRawPos.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "IntegSensVel");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simIntegSensVel.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "LimitFwd");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simFwdLimValue.set((int)value != 0);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFX.value, deviceID, "LimitRev");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFX.value, deviceID);
			if (err == 0) {
				m_simRevLimValue.set((int)value != 0);
			}
		}
	}

	// ------ set/get routines for WPILIB interfaces ------//
	/**
	 * Common interface for setting the speed of a simple speed controller.
	 *
	 * @param speed The speed to set.  Value should be between -1.0 and 1.0.
	 * 									Value is also saved for Get().
	 */
	@Override
	public void set(double speed) {
		_speed = speed;
		set(ControlMode.PercentOutput, _speed);
		feed();
	}

	/**
	 * Common interface for getting the current set speed of a speed controller.
	 *
	 * @return The current set speed. Value is between -1.0 and 1.0.
	 */
	@Override
	public double get() {
		return _speed;
	}

	// ---------Intercept CTRE calls for motor safety ---------//
	/**
	 * Sets the appropriate output on the talon, depending on the mode.
	 * @param mode The output mode to apply.
	 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
	 * In Current mode, output value is in amperes.
	 * In Velocity mode, output value is in position change / 100ms.
	 * In Position mode, output value is in encoder ticks or an analog value,
	 *   depending on the sensor.
	 * In Follower mode, the output value is the integer device ID of the talon to
	 * duplicate.
	 *
	 * @param value The setpoint value, as described above.
	 *
	 *
	 *	Standard Driving Example:
	 *	_talonLeft.set(ControlMode.PercentOutput, leftJoy);
	 *	_talonRght.set(ControlMode.PercentOutput, rghtJoy);
	 */
	public void set(ControlMode mode, double value) {
		/* intercept the advanced Set and feed motor-safety */
		super.set(mode, value);
		feed();
	}

	/**
	 * @param mode Sets the appropriate output on the talon, depending on the mode.
	 * @param demand0 The output value to apply.
	 * 	such as advanced feed forward and/or auxiliary close-looping in firmware.
	 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
	 * In Current mode, output value is in amperes.
	 * In Velocity mode, output value is in position change / 100ms.
	 * In Position mode, output value is in encoder ticks or an analog value,
	 *   depending on the sensor. See
	 * In Follower mode, the output value is the integer device ID of the talon to
	 * duplicate.
	 *
	 * @param demand1Type The demand type for demand1.
	 * Neutral: Ignore demand1 and apply no change to the demand0 output.
	 * AuxPID: Use demand1 to set the target for the auxiliary PID 1.
	 * ArbitraryFeedForward: Use demand1 as an arbitrary additive value to the
	 *	 demand0 output.  In PercentOutput the demand0 output is the motor output,
	 *   and in closed-loop modes the demand0 output is the output of PID0.
	 * @param demand1 Supplmental output value.  Units match the set mode.
	 *
	 *
	 *  Arcade Drive Example:
	 *		_talonLeft.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, +joyTurn);
	 *		_talonRght.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, -joyTurn);
	 *
	 *	Drive Straight Example:
	 *	Note: Selected Sensor Configuration is necessary for both PID0 and PID1.
	 *		_talonLeft.follow(_talonRght, FollwerType.AuxOutput1);
	 *		_talonRght.set(ControlMode.PercentOutput, joyForward, DemandType.AuxPID, desiredRobotHeading);
	 *
	 *	Drive Straight to a Distance Example:
	 *	Note: Other configurations (sensor selection, PID gains, etc.) need to be set.
	 *		_talonLeft.follow(_talonRght, FollwerType.AuxOutput1);
	 *		_talonRght.set(ControlMode.MotionMagic, targetDistance, DemandType.AuxPID, desiredRobotHeading);
	 */
	public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1){
		/* intercept the advanced Set and feed motor-safety */
		super.set(mode, demand0, demand1Type, demand1);
		feed();
	}

	/**
	 * Sets the voltage output of the SpeedController.  Compensates for the current bus
	 * voltage to ensure that the desired voltage is output even if the battery voltage is below
	 * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
	 * feedforward calculation).
	 *
	 * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
	 * properly - unlike the ordinary set function, it is not "set it and forget it."
	 *
	 * @param outputVolts The voltage to output.
	 */
	@Override
	public void setVoltage(double outputVolts) {
		if(super.isVoltageCompensationEnabled())
		{
			com.ctre.phoenix.Logger.log(ErrorCode.DoubleVoltageCompensatingWPI, _description + ": setVoltage ");
		}
		set(outputVolts / RobotController.getBatteryVoltage());
	}

	// ----------------------- Invert routines -------------------//
	/**
	 * Common interface for inverting direction of a speed controller.
	 *
	 * @param isInverted The state of inversion, true is inverted.
	 */
	@Override
	public void setInverted(boolean isInverted) {
		super.setInverted(isInverted);
	}

	/**
	 * Common interface for returning the inversion state of a speed controller.
	 *
	 * @return The state of inversion, true is inverted.
	 */
	@Override
	public boolean getInverted() {
		return super.getInverted();
	}

	// ----------------------- turn-motor-off routines-------------------//
	/**
	 * Common interface for disabling a motor.
	 */
	@Override
	public void disable() {
		neutralOutput();
	}

	/**
	 * Common interface to stop the motor until Set is called again.
	 */
	@Override
	public void stopMotor() {
		neutralOutput();
	}

	// ---- Sendable -------//

	/**
	 * Initialize sendable
	 * @param builder Base sendable to build on
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Motor Controller");
		builder.setActuator(true);
		builder.setSafeState(this::stopMotor);
		builder.addDoubleProperty("Value", this::get, this::set);
	}

	/**
	 * @return description of controller
	 */
	public String getDescription() {
		return _description;
	}

	/* ----- Motor Safety ----- */
	/** caller must lock appropriately */
	private WPI_MotorSafetyImplem GetMotorSafety() {
		if (_motorSafety == null) {
			/* newly created MS object */
			_motorSafety = new WPI_MotorSafetyImplem(this, getDescription());
			/* apply the expiration timeout */
			_motorSafety.setExpiration(_motSafeExpiration);
		}
		return _motorSafety;
	}
	/**
	 * Feed the motor safety object.
	 *
	 * <p>Resets the timer on this object that is used to do the timeouts.
	 */
	public void feed() {
		synchronized (_lockMotorSaf) {
			if (_motorSafety == null) {
				/* do nothing, MS features were never enabled */
			} else {
				GetMotorSafety().feed();
			}
		}
	}

	/**
	 * Set the expiration time for the corresponding motor safety object.
	 *
	 * @param expirationTime The timeout value in seconds.
	 */
	public void setExpiration(double expirationTime) {
		synchronized (_lockMotorSaf) {
			/* save the value for if/when we do create the MS object */
			_motSafeExpiration = expirationTime;
			/* apply it only if MS object exists */
			if (_motorSafety == null) {
				/* do nothing, MS features were never enabled */
			} else {
				/* this call will trigger creating a registered MS object */
				GetMotorSafety().setExpiration(_motSafeExpiration);
			}
		}
	}

	/**
	 * Retrieve the timeout value for the corresponding motor safety object.
	 *
	 * @return the timeout value in seconds.
	 */
	public double getExpiration() {
		synchronized (_lockMotorSaf) {
			/* return the intended expiration time */
			return _motSafeExpiration;
		}
	}

	/**
	 * Determine of the motor is still operating or has timed out.
	 *
	 * @return a true value if the motor is still operating normally and hasn't timed out.
	 */
	public boolean isAlive() {
		synchronized (_lockMotorSaf) {
			if (_motorSafety == null) {
				/* MC is alive - MS features were never enabled to neutral the MC. */
				return true;
			} else {
				return GetMotorSafety().isAlive();
			}
		}
	}

	/**
	 * Enable/disable motor safety for this device.
	 *
	 * <p>Turn on and off the motor safety option for this PWM object.
	 *
	 * @param enabled True if motor safety is enforced for this object
	 */
	public void setSafetyEnabled(boolean enabled) {
		synchronized (_lockMotorSaf) {
			if (_motorSafety == null && !enabled) {
				/* Caller wants to disable MS,
					but MS features were nevere enabled,
					so it doesn't need to be disabled. */
			} else {
				/* MS will be created if it does not exist */
				GetMotorSafety().setSafetyEnabled(enabled);
			}
		}
	}

	/**
	 * Return the state of the motor safety enabled flag.
	 *
	 * <p>Return if the motor safety is currently enabled for this device.
	 *
	 * @return True if motor safety is enforced for this device
	 */
	public boolean isSafetyEnabled() {
		synchronized (_lockMotorSaf) {
			if (_motorSafety == null) {
				/* MS features were never enabled. */
				return false;
			} else {
				return GetMotorSafety().isSafetyEnabled();
			}
		}
	}
}
