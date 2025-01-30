package org.team1540.robot2025.subsystems.elevator;

import static org.team1540.robot2025.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final MotionMagicVoltage profiledPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

    // Leader Elevator Motor
    private final TalonFX leader = new TalonFX(LEADER_ID);
    private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
    private final StatusSignal<Angle> leaderPosition = leader.getPosition();
    private final StatusSignal<Voltage> leaderAppliedVoltage = leader.getMotorVoltage();
    private final StatusSignal<Current> leaderCurrentAmps = leader.getSupplyCurrent();
    private final StatusSignal<Temperature> leaderTempCelsius = leader.getDeviceTemp();

    // Follower Elevator Motor
    private final TalonFX follower = new TalonFX(FOLLOWER_ID);
    private final StatusSignal<Voltage> followerAppliedVoltage = follower.getMotorVoltage();
    private final StatusSignal<Current> followerCurrentAmps = follower.getSupplyCurrent();
    private final StatusSignal<Temperature> followerTempCelsius = follower.getDeviceTemp();


    private final Follower followerControl = new Follower(LEADER_ID, true);
    private final DigitalInput upperLimitSwitch = new DigitalInput(UPPER_LIMIT_ID);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_ID);

    public ElevatorIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = MOTOR_ROTS_PER_METER;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_TIME_THRESHOLD;

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

        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;

        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();
        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);
        follower.setControl(followerControl);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                leaderPosition,
                leaderVelocity,
                leaderAppliedVoltage,
                followerAppliedVoltage,
                leaderCurrentAmps,
                followerCurrentAmps,
                leaderTempCelsius,
                followerTempCelsius);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderAppliedVoltage,
                followerAppliedVoltage,
                leaderCurrentAmps,
                followerCurrentAmps,
                leaderTempCelsius,
                followerTempCelsius);

        inputs.leaderCurrentAmps = leaderCurrentAmps.getValue().magnitude();
        inputs.leaderAppliedVolts = leaderAppliedVoltage.getValue().magnitude();
        inputs.leaderTempCelsius = leaderTempCelsius.getValue().magnitude();

        inputs.followerCurrentAmps = followerCurrentAmps.getValue().magnitude();
        inputs.followerAppliedVolts = followerAppliedVoltage.getValue().magnitude();
        inputs.followerTempCelsius = followerTempCelsius.getValue().magnitude();

        inputs.positionMeters = leaderPosition.getValue().magnitude();
        inputs.velocityMPS = leaderVelocity.getValue().magnitude();
        inputs.atUpperLimit = upperLimitSwitch.get();
        inputs.atLowerLimit = lowerLimitSwitch.get();
    }

    public void setVoltage(double volts) {
        leader.setVoltage(volts);
        follower.setControl(followerControl);
    }

    public void setSetpointMeters(double setpoint) {
        leader.setControl(profiledPositionControl.withPosition(setpoint));
        follower.setControl(followerControl);
    }

    public void setBrakeMode(boolean brakeMode) {
        if (brakeMode) leader.setNeutralMode(NeutralModeValue.Brake);
        else leader.setNeutralMode(NeutralModeValue.Coast);
        follower.setControl(followerControl);
    }
}
