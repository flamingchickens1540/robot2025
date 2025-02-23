package org.team1540.robot2025.util;

import static org.team1540.robot2025.FieldConstants.ReefHeight;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoard {
    private static final int AXIS_STEP = 20;
    private static final int BRANCH_FACE_AXIS_ID = 0;
    private static final int BRANCH_HEIGHT_AXIS_ID = 1;
    // Not implementing bc Jack said not needed yet. If needed, implement in same way as other two, left to
    // right is increasing starting from 0.
    private static final int INTAKE_LANE_AXIS_ID = 2;
    private static final int ALGAE_REEF_ACTION_AXIS_ID = 3;
    private static final int ALGAE_SCORE_ACTION_AXIS_ID = 4;

    public enum ReefButton {
        H,
        G,
        F,
        E,
        D,
        C,
        B,
        A,
        L,
        K,
        J,
        I;

        private static ReefButton fromOrdinal(int value) {
            return values()[value];
        }
    }

    private int getAxisState(int axis) {
        return (((int) Math.ceil(((hid.getHID().getRawAxis(axis) + 1) * 128))) / AXIS_STEP);
    }

    public final CommandGenericHID hid;

    public ButtonBoard(int port) {
        hid = new CommandGenericHID(port);
    }

    public Trigger branchHeightAt(ReefHeight level) {
        return new Trigger(
                CommandScheduler.getInstance().getDefaultButtonLoop(),
                () -> this.getAxisState(BRANCH_HEIGHT_AXIS_ID) == level.ordinal() && hid.isConnected());
    }

    public ReefHeight getSelectedBranchHeight() {
        return ReefHeight.fromLevel(this.getAxisState(BRANCH_HEIGHT_AXIS_ID) + 1);
    }

    public Trigger branchFaceAt(ReefButton face) {
        return new Trigger(
                CommandScheduler.getInstance().getDefaultButtonLoop(),
                () -> this.getAxisState(BRANCH_FACE_AXIS_ID) == face.ordinal() && hid.isConnected());
    }

    public ReefButton getSelectedBranchFace() {
        return ReefButton.fromOrdinal(this.getAxisState(BRANCH_FACE_AXIS_ID));
    }
}
