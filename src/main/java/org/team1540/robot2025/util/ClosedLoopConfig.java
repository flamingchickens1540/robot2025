package org.team1540.robot2025.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import org.team1540.robot2025.Constants;

public class ClosedLoopConfig {
    public enum GravityFFType {
        NONE,
        ARM,
        ELEVATOR
    }

    // PID constants
    public final double kP;
    public final double kI;
    public final double kD;

    // Feedforward constants
    public final double kS;
    public final double kV;
    public final double kA;
    public final double kG;

    public final GravityFFType gravityFFType;

    public ClosedLoopConfig(
            double kP, double kI, double kD, double kS, double kV, double kA, double kG, GravityFFType gravityFFType) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        this.gravityFFType = gravityFFType;
    }

    public ClosedLoopConfig(double kP, double kI, double kD, double kS, double kV, double kA) {
        this(kP, kI, kD, kS, kV, kA, 0, GravityFFType.NONE);
    }

    public ClosedLoopConfig(double kP, double kI, double kD, double kS, double kV) {
        this(kP, kI, kD, kS, kV, 0);
    }

    public ClosedLoopConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0);
    }

    public ClosedLoopConfig(PIDController pid, SimpleMotorFeedforward ff) {
        this(pid.getP(), pid.getI(), pid.getD(), ff.getKs(), ff.getKv(), ff.getKa());
    }

    public ClosedLoopConfig(PIDController pid, ArmFeedforward ff) {
        this(pid.getP(), pid.getI(), pid.getD(), ff.getKs(), ff.getKv(), ff.getKa(), ff.getKg(), GravityFFType.ARM);
    }

    public ClosedLoopConfig(Slot0Configs phoenixConfigs) {
        this(
                phoenixConfigs.kP,
                phoenixConfigs.kI,
                phoenixConfigs.kD,
                phoenixConfigs.kS,
                phoenixConfigs.kV,
                phoenixConfigs.kA,
                phoenixConfigs.kG,
                phoenixConfigs.kG == 0
                        ? GravityFFType.NONE
                        : switch (phoenixConfigs.GravityType) {
                            case Arm_Cosine -> GravityFFType.ARM;
                            case Elevator_Static -> GravityFFType.ELEVATOR;
                        });
    }

    public ClosedLoopConfig(PIDController pid, ElevatorFeedforward ff) {
        this(
                pid.getP(),
                pid.getI(),
                pid.getD(),
                ff.getKs(),
                ff.getKv(),
                ff.getKa(),
                ff.getKg(),
                GravityFFType.ELEVATOR);
    }

    public PIDController createPIDController(double loopPeriodSecs) {
        return new PIDController(kP, kI, kD, loopPeriodSecs);
    }

    public PIDController createPIDController() {
        return createPIDController(Constants.kLoopPeriodSecs);
    }

    public ProfiledPIDController createProfiledPIDController(TrapezoidProfile.Constraints constraints) {
        return new ProfiledPIDController(kP, kI, kD, constraints);
    }

    public SimpleMotorFeedforward createMotorFF() {
        return new SimpleMotorFeedforward(kS, kV, kA);
    }

    public ArmFeedforward createArmFF() {
        if (gravityFFType != GravityFFType.ARM) {
            DriverStation.reportError("Trying to create arm feedforward with non-arm config", true);
        }
        return new ArmFeedforward(kS, kG, kV, kA);
    }

    public ElevatorFeedforward createElevatorFF() {
        if (gravityFFType != GravityFFType.ELEVATOR) {
            DriverStation.reportError("Trying to create elevator feedforward with non-elevator config", true);
        }
        return new ElevatorFeedforward(kS, kG, kV, kA);
    }

    public Slot0Configs createCTREConfigs() {
        Slot0Configs configs = new Slot0Configs();
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;

        configs.kS = kS;
        configs.kV = kV;
        configs.kA = kA;

        switch (gravityFFType) {
            case ARM -> {
                configs.kG = kG;
                configs.GravityType = GravityTypeValue.Arm_Cosine;
            }
            case ELEVATOR -> {
                configs.kG = kG;
                configs.GravityType = GravityTypeValue.Elevator_Static;
            }
            case NONE -> configs.kG = 0;
        }
        return configs;
    }
}
