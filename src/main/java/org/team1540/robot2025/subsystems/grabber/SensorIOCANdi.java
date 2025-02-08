package org.team1540.robot2025.subsystems.grabber;

import static org.team1540.robot2025.subsystems.grabber.GrabberConstants.CANDI_ID;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;

public class SensorIOCANdi implements SensorIO {
    private final CANdi candi = new CANdi(CANDI_ID);

    public SensorIOCANdi() {
        DigitalInputsConfigs digitalInputsConfig = new DigitalInputsConfigs();
        // TODO: Fix these sometime
        digitalInputsConfig
                .withS1CloseState(S1CloseStateValue.CloseWhenFloating)
                .withS2CloseState(S2CloseStateValue.CloseWhenFloating);
        CANdiConfiguration candiConfig = new CANdiConfiguration();
        candiConfig.withDigitalInputs(digitalInputsConfig);
    }

    @Override
    public void updateInputs(SensorIOInputs inputs) {
        inputs.beforeSensorTripped = candi.getS1Closed().getValue();
        inputs.afterSensorTripped = candi.getS2Closed().getValue();

        inputs.beforeSensorConnected = candi.isConnected();
        inputs.afterSensorConnected = candi.isConnected();
    }
}
