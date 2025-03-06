package org.team1540.robot2025.services;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.generated.TunerConstants;
import org.team1540.robot2025.util.LoggedTracer;

/**
 * Class for managing high-level robot alerts, such as low battery and CAN bus errors.
 * Subsystem-specific alerts, such as motor disconnects, should be handled by their respective
 * subsystems.
 */
public class AlertManager {
    private static AlertManager instance;

    public static AlertManager getInstance() {
        if (instance == null) instance = new AlertManager();
        return instance;
    }

    private static final double lowBatteryVoltageThreshold = 12.0;

    private final Debouncer canErrorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
    private final Debouncer canivoreErrorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
    private final Debouncer lowBatteryDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
    private final Debouncer lowBatteryDisabledDebouncer = new Debouncer(1.5, Debouncer.DebounceType.kRising);

    private int lastRioCanTEC = 0;
    private int lastRioCanREC = 0;
    private int lastCanivoreTEC = 0;
    private int lastCanivoreREC = 0;

    private final Alert canErrorAlert =
            new Alert("RIO CAN bus error, robot functionality may be severely limited", Alert.AlertType.kError);
    private final Alert canivoreErrorAlert =
            new Alert("Swerve CAN bus error, drive functionality may be severely limited", Alert.AlertType.kError);
    private final Alert lowBatteryAlert =
            new Alert("Battery voltage is low, consider replacing it", Alert.AlertType.kWarning);
    private final Alert tuningModeAlert = new Alert("Tuning mode is enabled", Alert.AlertType.kInfo);

    public void update() {
        LoggedTracer.reset();

        tuningModeAlert.set(Constants.isTuningMode());

        // Update CAN bus alerts
        CANStatus rioCanStatus = RobotController.getCANStatus();
        boolean rioCanError =
                rioCanStatus.transmitErrorCount > lastRioCanTEC || rioCanStatus.receiveErrorCount > lastRioCanREC;
        lastRioCanTEC = rioCanStatus.transmitErrorCount;
        lastRioCanREC = rioCanStatus.receiveErrorCount;

        CANBus.CANBusStatus canivoreStatus = TunerConstants.kCANBus.getStatus();
        boolean canivoreError = !canivoreStatus.Status.isOK()
                || canivoreStatus.TEC > lastCanivoreTEC
                || canivoreStatus.REC > lastCanivoreREC;
        lastCanivoreTEC = canivoreStatus.TEC;
        lastCanivoreREC = canivoreStatus.REC;

        canErrorAlert.set(canErrorDebouncer.calculate(rioCanError));
        canivoreErrorAlert.set(canivoreErrorDebouncer.calculate(canivoreError));
        lowBatteryAlert.set(
                lowBatteryDebouncer.calculate(RobotController.getBatteryVoltage() < lowBatteryVoltageThreshold)
                        && lowBatteryDisabledDebouncer.calculate(DriverStation.isDisabled()));

        LoggedTracer.record("AlertManager");
    }
}
