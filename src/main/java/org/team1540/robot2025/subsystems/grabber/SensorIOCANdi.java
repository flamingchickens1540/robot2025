package org.team1540.robot2025.subsystems.grabber;

import static org.team1540.robot2025.subsystems.grabber.GrabberConstants.CANDI_ID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;

public class SensorIOCANdi implements SensorIO {
    private final CANdi candi = new CANdi(CANDI_ID);

    private final StatusSignal<Boolean> beforeSensorTripped = candi.getS1Closed();
    private final StatusSignal<Boolean> afterSensorTripped = candi.getS2Closed();

    public SensorIOCANdi() {
        DigitalInputsConfigs digitalInputsConfig = new DigitalInputsConfigs();
        digitalInputsConfig
                .withS1CloseState(S1CloseStateValue.CloseWhenFloating)
                .withS2CloseState(S2CloseStateValue.CloseWhenFloating);
        CANdiConfiguration candiConfig = new CANdiConfiguration();
        candiConfig.withDigitalInputs(digitalInputsConfig);
        candi.getConfigurator().apply(candiConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, beforeSensorTripped, afterSensorTripped);
        candi.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SensorIOInputs inputs) {
        inputs.beforeSensorConnected = BaseStatusSignal.refreshAll(beforeSensorTripped, afterSensorTripped)
                .isOK();
        inputs.afterSensorConnected = inputs.beforeSensorConnected;

        inputs.beforeSensorTripped = beforeSensorTripped.getValue();
        inputs.afterSensorTripped = afterSensorTripped.getValue();
    }
}
