package org.team1540.robot2025.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        StatusCode error = StatusCode.OK;
        for (int i = 0; i < maxAttempts; i++) {
            error = command.get();
            if (error.isOK()) return error;
        }
        DriverStation.reportWarning(
                "Failed to run a command on a CTRE device after " + maxAttempts + " attempts:\n" + "\t"
                        + error.getDescription(),
                true);
        return error;
    }
}
