package org.team1540.robot2025.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static org.team1540.robot2025.Constants.Arm.*;

public class ArmIOReal implements ArmIO{
    private final TalonFX motor = new TalonFX(MOTOR_ID);
    private final CANcoder cancoder = new CANcoder(CANCODER_ID);

   //TODO: write status signals later after figuring out what methods I want

//    private final StatusSignal<Double> position = motor.getPosition();

    public ArmIOReal(){
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        //TODO: clockwise positive or counter clockwise positive?
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = CANCODER_ID;
        motorConfig.Feedback.SensorToMechanismRatio = CANCODER_TO_PIVOT_RATIO*MOTOR_TO_CANCODER;

    }
    @Override
    public void updateInputs(ArmIOInputs inputs){

    }
//TODO:
//    @Override
//    public void setPosition
}
