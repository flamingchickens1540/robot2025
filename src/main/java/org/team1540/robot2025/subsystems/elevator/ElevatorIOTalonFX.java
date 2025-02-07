package org.team1540.robot2025.subsystems.elevator;

import static org.team1540.robot2025.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final MotionMagicVoltage profiledPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

    // Leader Elevator Motor
    private final TalonFX leader = new TalonFX(LEADER_ID);
    private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
    private final StatusSignal<Angle> leaderPosition = leader.getPosition();
    private final StatusSignal<Voltage> leaderAppliedVoltage = leader.getMotorVoltage();
    private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
    private final StatusSignal<Temperature> leaderTemp = leader.getDeviceTemp();
    private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();

    // Follower Elevator Motor
    private final TalonFX follower = new TalonFX(FOLLOWER_ID);
    private final StatusSignal<Voltage> followerAppliedVoltage = follower.getMotorVoltage();
    private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent();
    private final StatusSignal<Temperature> followerTemp = follower.getDeviceTemp();
    private final StatusSignal<Angle> followerPosition = follower.getPosition();
    private final StatusSignal<AngularVelocity> followerVelocity = follower.getVelocity();
    private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();

    private final Follower followerControl = new Follower(LEADER_ID, true);
    private final DigitalInput upperLimitSwitch = new DigitalInput(UPPER_LIMIT_ID);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_ID);

    private final Debouncer leaderDebouncer = new Debouncer(0.5);
    private final Debouncer followerDebouncer = new Debouncer(0.5);

    public ElevatorIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = MOTOR_ROTS_PER_METER;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        config.Slot0.kP = KP;
        config.Slot0.kI = KI;
        config.Slot0.kD = KD;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kG = KG;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        config.MotionMagic.MotionMagicCruiseVelocity = 1;
        config.MotionMagic.MotionMagicAcceleration = 2;

        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);
        follower.setControl(followerControl);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                leaderPosition,
                followerPosition,
                leaderVelocity,
                followerVelocity,
                leaderAppliedVoltage,
                followerAppliedVoltage,
                leaderSupplyCurrent,
                followerSupplyCurrent,
                leaderStatorCurrent,
                followerStatorCurrent,
                leaderTemp,
                followerTemp);

        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        StatusCode leaderStatus = BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderAppliedVoltage,
                leaderSupplyCurrent,
                leaderStatorCurrent,
                leaderTemp);
        StatusCode followerStatus = BaseStatusSignal.refreshAll(
                followerPosition,
                followerVelocity,
                followerAppliedVoltage,
                followerSupplyCurrent,
                followerStatorCurrent,
                followerTemp);
        inputs.connection = new boolean[] {
            leaderDebouncer.calculate(leaderStatus.isOK()), followerDebouncer.calculate(leaderStatus.isOK())
        };
        inputs.supplyCurrentAmps =
                new double[] {leaderSupplyCurrent.getValueAsDouble(), followerSupplyCurrent.getValueAsDouble()};
        inputs.appliedVolts =
                new double[] {leaderAppliedVoltage.getValueAsDouble(), followerAppliedVoltage.getValueAsDouble()};
        inputs.tempCelsius = new double[] {leaderTemp.getValueAsDouble(), followerTemp.getValueAsDouble()};
        inputs.statorCurrentAmps =
                new double[] {leaderStatorCurrent.getValueAsDouble(), followerStatorCurrent.getValueAsDouble()};
        inputs.positionMeters = new double[] {leaderPosition.getValueAsDouble(), followerPosition.getValueAsDouble()};
        inputs.velocityMPS = new double[] {leaderVelocity.getValueAsDouble(), followerVelocity.getValueAsDouble()};
        inputs.atUpperLimit = upperLimitSwitch.get();
        inputs.atLowerLimit = lowerLimitSwitch.get();
    }

    public void setVoltage(double volts) {
        leader.setVoltage(volts);
        follower.setControl(followerControl);
    }

    public void setSetpoint(double setpointMeters) {
        leader.setControl(profiledPositionControl.withPosition(setpointMeters));
        follower.setControl(followerControl);
    }

    public void setBrakeMode(boolean brakeMode) {
        leader.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        follower.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        // follower.setControl(followerControl);
    }

    public void configPID(double kP, double kI, double kD) {
        Slot0Configs configs = new Slot0Configs();
        leader.getConfigurator().refresh(configs);
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        leader.getConfigurator().apply(configs);
        follower.setControl(followerControl);
    }

    public void configFF(double kS, double kV, double kG) {
        Slot0Configs configs = new Slot0Configs();
        leader.getConfigurator().refresh(configs);
        configs.kS = kS;
        configs.kV = kV;
        configs.kG = kG;
        leader.getConfigurator().apply(configs);
        follower.setControl(followerControl);
    }
}
