package org.team1540.robot2025.subsystems.grabber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

import static org.team1540.robot2025.subsystems.grabber.GrabberConstants.*;


public class GrabberIOTalonFX implements GrabberIO {
    private final TalonFX motor = new TalonFX(GRABBER_ID);
    //TODO: Fix candi configuration stuff
    private final CANdi beforeBreak = new CANdi(BEFORE_BREAK_CANDI_ID, CANDI_BUS);
    private final CANdi afterBreak = new CANdi(AFTER_BREAK_CANDI_ID, CANDI_BUS);

    private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage();
    private final StatusSignal<Temperature> motorTemp = motor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
    private final StatusSignal<Angle> motorAngle = motor.getPosition();
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true);

    public GrabberIOTalonFX() {
        beforeBreak.getConfigurator().apply(new CANdiConfiguration());
        afterBreak.getConfigurator().apply(new CANdiConfiguration());

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //TODO: Change all this maybe
        motorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        motor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                motorCurrent,
                motorVoltage,
                motorTemp,
                motorVelocity,
                motorAngle
        );

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                motorCurrent,
                motorVoltage,
                motorTemp,
                motorVelocity,
                motorAngle
        );

        inputs.motorCurrentAmps = motorCurrent.getValueAsDouble();
        inputs.motorAppliedVolts = motorVoltage.getValueAsDouble();
        inputs.motorTempCelsius = motorTemp.getValueAsDouble();
        inputs.velocityRPM = motorVelocity.getValueAsDouble();
        inputs.positionRots = motorAngle.getValueAsDouble();

        inputs.hasCoral = beforeBreak.getS1Closed().getValue() && afterBreak.getS2Closed().getValue(); //TODO: fix beam break logic
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setVelocity(double velocityRPM) {
        motor.setControl(velocityControl.withVelocity(velocityRPM));
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        motor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
