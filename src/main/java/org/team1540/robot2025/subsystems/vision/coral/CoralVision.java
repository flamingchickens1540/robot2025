package org.team1540.robot2025.subsystems.vision.coral;

import static org.team1540.robot2025.subsystems.vision.coral.CoralVisionConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.RobotState;

public class CoralVision extends SubsystemBase {
    private final CoralVisionIO io;
    private final CoralVisionIOInputsAutoLogged inputs = new CoralVisionIOInputsAutoLogged();
    private final Alert disconnectedAlert;

    private CoralVision(CoralVisionIO io) {
        this.io = io;
        this.disconnectedAlert = new Alert(io.getName() + " is disconnected oh noes", Alert.AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralVision", inputs);

        if (inputs.hasDetection) RobotState.getInstance().addCoralObservation(inputs.latestObservation);

        disconnectedAlert.set(!inputs.connected);
    }

    public boolean hasCoralDetection() {
        return inputs.hasDetection;
    }

    public static CoralVision createReal() {
        return new CoralVision(new CoralVisionIOLimelight(CAMERA_NAME));
    }

    public static CoralVision createDummy() {
        return new CoralVision(new CoralVisionIO() {});
    }
}
