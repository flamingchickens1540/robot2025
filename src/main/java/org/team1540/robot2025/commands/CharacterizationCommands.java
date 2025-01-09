package org.team1540.robot2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.util.LoggedTunableNumber;
import org.team1540.robot2025.util.math.PolynomialRegression;

public class CharacterizationCommands {
    private static final double kStartDelaySecs = 1.0;

    private static final LoggedTunableNumber ffRampVoltsPerSec =
            new LoggedTunableNumber("Characterization/Feedforward/RampVoltsPerSec", 0.1);

    public static Command feedforward(
            DoubleConsumer voltageConsumer, DoubleSupplier velocitySupplier, Subsystem... requirements) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        var command = Commands.sequence(
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),
                Commands.run(() -> voltageConsumer.accept(0.0)).withTimeout(kStartDelaySecs),
                Commands.runOnce(timer::restart),
                Commands.run(() -> {
                            double voltage = timer.get() * ffRampVoltsPerSec.get();
                            voltageConsumer.accept(voltage);
                            velocitySamples.add(velocitySupplier.getAsDouble());
                            voltageSamples.add(voltage);
                        })
                        .finallyDo(() -> {
                            if (velocitySamples.isEmpty() || voltageSamples.isEmpty()) {
                                return;
                            }

                            PolynomialRegression regression = new PolynomialRegression(
                                    velocitySamples.stream()
                                            .mapToDouble(Double::doubleValue)
                                            .toArray(),
                                    voltageSamples.stream()
                                            .mapToDouble(Double::doubleValue)
                                            .toArray(),
                                    1);

                            System.out.println("********** Feedforward Characterization Results **********");
                            System.out.println("\tCount=" + velocitySamples.size());
                            System.out.printf("\tR2=%.5f%n", regression.R2());
                            System.out.printf("\tkS=%.5f%n", regression.beta(0));
                            System.out.printf("\tkV=%.5f%n", regression.beta(1));

                            Logger.recordOutput("Characterization/Feedforward/SampleCount", velocitySamples.size());
                            Logger.recordOutput("Characterization/Feedforward/R2", regression.R2());
                            Logger.recordOutput("Characterization/Feedforward/kS", regression.beta(0));
                            Logger.recordOutput("Characterization/Feedforward/kV", regression.beta(1));
                        }));
        command.addRequirements(requirements);
        return command;
    }

    private static final LoggedTunableNumber wheelRadiusSpeedRadsPerSec =
            new LoggedTunableNumber("Characterization/WheelRadius/SpeedRadsPerSec", 0.5);
    private static final LoggedTunableNumber wheelRadiusRampRate =
            new LoggedTunableNumber("Characterization/WheelRadius/RampRate", 0.5);

    public static Command wheelRadius(
            DoubleConsumer omegaConsumer,
            DoubleSupplier gyroYawRadsSupplier,
            Supplier<double[]> wheelPositionsRadsSupplier,
            Subsystem... requirements) {
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        var command = Commands.parallel(
                        Commands.sequence(
                                Commands.runOnce(() -> state.limiter = new SlewRateLimiter(wheelRadiusRampRate.get())),
                                Commands.run(() -> {
                                    double omega = state.limiter.calculate(wheelRadiusSpeedRadsPerSec.get());
                                    omegaConsumer.accept(omega);
                                })),
                        Commands.sequence(
                                Commands.waitSeconds(kStartDelaySecs),
                                Commands.runOnce(() -> {
                                    state.startPositions = wheelPositionsRadsSupplier.get();
                                    state.lastYawRads = gyroYawRadsSupplier.getAsDouble();
                                    state.accumGyroYawRads = 0.0;
                                }),
                                Commands.run(() -> {
                                    double rotation = gyroYawRadsSupplier.getAsDouble();
                                    state.accumGyroYawRads +=
                                            Math.abs(MathUtil.angleModulus(rotation - state.lastYawRads));
                                    state.lastYawRads = rotation;
                                })))
                .finallyDo(() -> {
                    double[] positions = wheelPositionsRadsSupplier.get();
                    double wheelDelta = 0.0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.startPositions[i]);
                    }
                    wheelDelta /= 4.0;
                    double effectiveRadius = (state.accumGyroYawRads * Drivetrain.kDrivebaseRadius) / wheelDelta;
                    effectiveRadius = Units.metersToInches(effectiveRadius);

                    System.out.println("********** Wheel Radius Characterization Results **********");
                    System.out.println("\tAvg Wheel Delta: " + wheelDelta);
                    System.out.println("\tAccum Gyro Yaw: " + state.accumGyroYawRads);
                    System.out.println("\tEffective Radius (inches): " + effectiveRadius);

                    Logger.recordOutput("Characterization/WheelRadius/AvgWheelDelta", wheelDelta);
                    Logger.recordOutput("Characterization/WheelRadius/AccumGyroYaw", state.accumGyroYawRads);
                    Logger.recordOutput("Characterization/WheelRadius/EffectiveRadiusInches", effectiveRadius);
                });
        command.addRequirements(requirements);
        return command;
    }

    private static class WheelRadiusCharacterizationState {
        SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRate.get());
        double[] startPositions = new double[4];
        double lastYawRads = 0.0;
        double accumGyroYawRads = 0.0;
    }
}
