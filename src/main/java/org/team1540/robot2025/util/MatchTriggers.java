package org.team1540.robot2025.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MatchTriggers {
    public static Trigger endgame() {
        return new Trigger(() -> DriverStation.getMatchTime() <= 30)
                .and(DriverStation::isTeleopEnabled)
                .and(DriverStation::isFMSAttached);
    }
}
