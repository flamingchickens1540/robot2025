package org.team1540.robot2025.subsystems.arm;

import static org.team1540.robot2025.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmIOReal implements ArmIO {
    private final TalonFX motor = new TalonFX(MOTOR_ID);
    private final CANcoder cancoder = new CANcoder(CANCODER_ID);

    // TODO: write status signals later after figuring out what methods I want

    //    private final StatusSignal<Double> position = motor.getPosition();

    public ArmIOReal() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // TODO: clockwise positive or counter clockwise positive?
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = CANCODER_ID;
        motorConfig.Feedback.SensorToMechanismRatio = CANCODER_TO_PIVOT_RATIO * MOTOR_TO_CANCODER;
        motorConfig.Feedback.RotorToSensorRatio = 1; // TODO: is this even true?

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.getRotations();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MIN_ANGLE.getRotations();

        motorConfig.Slot0.kP = KP;
        motorConfig.Slot0.kI = KI;
        motorConfig.Slot0.kD = KD;
        motorConfig.Slot0.kS = KS;
        motorConfig.Slot0.kG = KG;
        motorConfig.Slot0.kV = KV;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        /*
        Note that the sensor offset and ratios must be
        configured so that the sensor reports a position
         of 0 when the mechanism is horizonal
         (parallel to the ground), and the reported
         sensor position is 1:1 with the mechanism.
         */

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCEL_RPS;
        motorConfig.MotionMagic.MotionMagicJerk = JERK_RPS;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = CURRENT_LOWER_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = TIME_LOWER_LIMIT;

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {}

    // TODO:
    //    @Override
    //    public void setPosition
}
