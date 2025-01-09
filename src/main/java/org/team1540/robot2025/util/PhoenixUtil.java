package org.team1540.robot2025.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) return;
        }
        DriverStation.reportWarning(
                "Failed to run a command on a CTRE device after " + maxAttempts + " attempts.", true);
    }
}
