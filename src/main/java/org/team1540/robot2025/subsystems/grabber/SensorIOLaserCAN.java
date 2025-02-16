package org.team1540.robot2025.subsystems.grabber;

import static au.grapplerobotics.interfaces.LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;
import static org.team1540.robot2025.subsystems.grabber.GrabberConstants.*;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;

public class SensorIOLaserCAN implements SensorIO {
    private final LaserCan beforeLaserCAN;
    private final LaserCan afterLaserCAN;

    public SensorIOLaserCAN() {
        beforeLaserCAN = new LaserCan(BEFORE_LASER_CAN_ID);
        afterLaserCAN = new LaserCan(AFTER_LASER_CAN_ID);
    }

    @Override
    public void updateInputs(SensorIOInputs inputs) {
        LaserCanInterface.Measurement beforeMeasurement = beforeLaserCAN.getMeasurement();
        inputs.beforeSensorConnected = beforeMeasurement != null;
        inputs.beforeSensorTripped = beforeMeasurement != null
                && beforeMeasurement.status == LASERCAN_STATUS_VALID_MEASUREMENT
                && beforeMeasurement.distance_mm <= LASER_CAN_DETECT_DISTANCE_MM;
        LaserCanInterface.Measurement afterMeasurement = afterLaserCAN.getMeasurement();
        inputs.afterSensorConnected = afterMeasurement != null;
        inputs.afterSensorTripped = afterMeasurement != null
                && afterMeasurement.status == LASERCAN_STATUS_VALID_MEASUREMENT
                && afterMeasurement.distance_mm <= LASER_CAN_DETECT_DISTANCE_MM;
    }
}
