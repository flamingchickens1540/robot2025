package org.team1540.robot2025.subsystems.grabber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Grabber extends SubsystemBase {
    private final GrabberIO grabberIO;
    private final SensorIO sensorIO;
    private final GrabberIOInputsAutoLogged grabberInputs = new GrabberIOInputsAutoLogged();
    private final SensorIOInputsAutoLogged sensorInputs = new SensorIOInputsAutoLogged();
    private final Alert motorDisconnectedAlert = new Alert("Grabber motor is disconnected", Alert.AlertType.kWarning);
    private final Alert beforeSensorDisconnectedAlert =
            new Alert("Before sensor is disconnected", Alert.AlertType.kWarning);
    private final Alert afterSensorDisconnectedAlert =
            new Alert("After sensor is disconnected", Alert.AlertType.kWarning);

    private Grabber(GrabberIO grabberIO, SensorIO sensorIO) {
        this.grabberIO = grabberIO;
        this.sensorIO = sensorIO;
    }

    @Override
    public void periodic() {
        grabberIO.updateInputs(grabberInputs);
        sensorIO.updateInputs(sensorInputs);

        Logger.processInputs("Grabber/motor", grabberInputs);
        Logger.processInputs("Grabber/sensor", sensorInputs);

        motorDisconnectedAlert.set(!grabberInputs.motorConnected);
        beforeSensorDisconnectedAlert.set(!sensorInputs.beforeSensorConnected);
        afterSensorDisconnectedAlert.set(!sensorInputs.afterSensorConnected);

        if (RobotState.isDisabled()) {
            stop();
        }
    }

    public void setPercent(double percent) {
        grabberIO.setVoltage(percent * 12.0);
    }

    public boolean hasCoral() {
        return sensorInputs.beforeSensorTripped && sensorInputs.afterSensorTripped;
    }

    public void stop() {
        grabberIO.setVoltage(0.0);
    }

    public double getStatorCurrent() {
        return grabberInputs.motorStatorCurrentAmps;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        grabberIO.setBrakeMode(isBrakeMode);
    }

    public Command commandRun(double percent) {
        return Commands.startEnd(() -> this.setPercent(percent), () -> this.setPercent(0), this);
    }

    public static Grabber createReal() {
        return new Grabber(new GrabberIOTalonFX(), new SensorIOCANdi());
    }

    public static Grabber createSim() {
        return new Grabber(new GrabberIOSim(), new SensorIO() {});
    }

    public static Grabber createDummy() {
        return new Grabber(new GrabberIO() {}, new SensorIO() {});
    }
}
