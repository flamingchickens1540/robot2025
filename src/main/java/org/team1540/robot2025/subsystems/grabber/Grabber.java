package org.team1540.robot2025.subsystems.grabber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static org.team1540.robot2025.subsystems.grabber.GrabberConstants.GRABBER_GEAR_RATIO;


public class Grabber extends SubsystemBase {
    private final GrabberIO io;
    private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

    private final PIDController positionPID = new PIDController(1.540, 0.0, 0.0);
    private boolean isClosedLoop = false;

    private Grabber(GrabberIO io) {
        this.io = io;
    }

    public static Grabber createReal() {
        return new Grabber(new GrabberIOTalonFX());
    }

    public static Grabber createSim() {
        return new Grabber(new GrabberIO() {});
    }

    public static Grabber createDummy() {
        return new Grabber(new GrabberIO() {});
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Grabber", inputs);

        if (isClosedLoop) {
            io.setVoltage(positionPID.calculate(inputs.positionRots));
        }

        if (RobotState.isDisabled()) {
            stop();
        }
    }

    public void setPercent(double percent) {
        isClosedLoop = false;
        io.setVoltage(percent * 12.0);
    }

    public void moveRotations(double rotations) {
        isClosedLoop = true;
        positionPID.setSetpoint(GRABBER_GEAR_RATIO * rotations + inputs.positionRots);
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    public Command commandRun(double percent) {
        return Commands.startEnd(
                () -> this.setPercent(percent),
                () -> this.setPercent(0),
                this
        );
    }
}
