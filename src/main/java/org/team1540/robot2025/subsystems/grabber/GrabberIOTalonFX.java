package org.team1540.robot2025.subsystems.grabber;

import static org.team1540.robot2025.subsystems.grabber.GrabberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;

public class GrabberIOTalonFX implements GrabberIO {
    private final TalonFX motor = new TalonFX(MOTOR_ID);

    private final StatusSignal<Current> motorSupplyCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Current> motorStatorCurrent = motor.getStatorCurrent();
    private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage();
    private final StatusSignal<Temperature> motorTemp = motor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
    private final StatusSignal<Angle> motorAngle = motor.getPosition();

    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);

    private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

    public GrabberIOTalonFX() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // TODO: Change all this maybe
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        motorConfig.CurrentLimits.StatorCurrentLimit = 100;

        motor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, motorSupplyCurrent, motorStatorCurrent, motorVoltage, motorTemp, motorVelocity, motorAngle);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        StatusCode motorStatus = BaseStatusSignal.refreshAll(
                motorSupplyCurrent, motorStatorCurrent, motorVoltage, motorTemp, motorVelocity, motorAngle);

        inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
        inputs.motorSupplyCurrentAmps = motorSupplyCurrent.getValueAsDouble();
        inputs.motorStatorCurrentAmps = motorStatorCurrent.getValueAsDouble();
        inputs.motorAppliedVolts = motorVoltage.getValueAsDouble();
        inputs.motorTempCelsius = motorTemp.getValueAsDouble();
        inputs.motorVelocityRPM = motorVelocity.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        motor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
