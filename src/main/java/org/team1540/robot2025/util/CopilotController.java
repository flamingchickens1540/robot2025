package org.team1540.robot2025.util;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;

public class CopilotController {

    public enum BranchHeight {
        L1(0),
        L2(45),
        L3(90),
        L4(135);
        private final int angle;

        BranchHeight(int povangle) {
            this.angle = povangle;
        }

        private static Optional<BranchHeight> fromAngle(int angle) {
            if (angle == L1.angle) return Optional.of(L1);
            if (angle == L2.angle) return Optional.of(L2);
            if (angle == L3.angle) return Optional.of(L3);
            if (angle == L4.angle) return Optional.of(L4);
            return Optional.empty();
        }
    }

    public final CommandGenericHID hid;

    public CopilotController(int port) {
        hid = new CommandGenericHID(port);
    }

    private static final int BRANCH_HEIGHT_POV_ID = 0;

    public Trigger branchHeightAt(BranchHeight level) {
        return hid.pov(
                BRANCH_HEIGHT_POV_ID,
                level.angle,
                CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Optional<BranchHeight> getSelectedBranch() {
        int angle = hid.getHID().getPOV(BRANCH_HEIGHT_POV_ID);
        return BranchHeight.fromAngle(angle);
    }
}
