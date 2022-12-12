// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.subsystems.Drive;
import frc.subsystems.SubsystemBase;
import frc.subsystems.Vision;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import java.util.List;

public class Robot extends LoggedRobot {

    private final List<SubsystemBase> subsystems = List.of(
            new Vision(),
            new Drive()
    );

    private RobotState state;
    private Commands commands;

    private OperatorInterface operatorInterface;

    @Override
    public void robotInit() {
        // Advantage scope
        Logger.getInstance().recordMetadata("ProjectName", "Robot 2023");

        if (isReal()) {
            Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        }
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        Logger.getInstance().start();

        state = new RobotState();
        commands = new Commands();
        operatorInterface = new OperatorInterface();

        subsystems.forEach(SubsystemBase::configure);
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {
        readSubsystems();
        updateApply();
    }

    @Override
    public void teleopInit() {
        commands.setDriveTeleop();
    }

    @Override
    public void teleopPeriodic() {
        readSubsystems();
        operatorInterface.update(commands, state);
        updateApply();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        readSubsystems();
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {
        readSubsystems();
        updateApply();
    }

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {
        subsystems.forEach(s -> s.simulate(state));
    }

    private void readSubsystems() {
        subsystems.forEach(s -> s.read(state));
        state.log();
    }

    private void updateApply() {
        subsystems.forEach(s -> s.update(commands, state));
        subsystems.forEach(SubsystemBase::write);
    }
}
