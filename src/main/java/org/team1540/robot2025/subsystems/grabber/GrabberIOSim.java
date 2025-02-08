package org.team1540.robot2025.subsystems.grabber;

import static org.team1540.robot2025.Constants.LOOP_PERIOD_SECS;
import static org.team1540.robot2025.subsystems.grabber.GrabberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class GrabberIOSim implements GrabberIO {
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(1), GRABBER_MOI, GRABBER_GEAR_RATIO),
            DCMotor.getFalcon500Foc(1));

    private double motorVoltage = 0.0;

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        sim.setInputVoltage(motorVoltage);
        sim.update(LOOP_PERIOD_SECS);
        inputs.motorSupplyCurrentAmps = sim.getCurrentDrawAmps();
        inputs.motorAppliedVolts = motorVoltage;
        inputs.velocityRPM = sim.getAngularVelocityRPM();
    }

    @Override
    public void setVoltage(double volts) {
        motorVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    }
}
