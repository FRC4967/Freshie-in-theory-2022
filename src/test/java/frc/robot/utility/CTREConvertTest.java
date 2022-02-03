package frc.robot.utility;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class CTREConvertTest {
    

    @Test
    public void testDistanceRoundTrip() {

        int sensorCounts = 8052;

        var meters = CTREConvert.nativeUnitsToDistanceMeters(sensorCounts);
        var calculatedCounts = CTREConvert.distanceToNativeUnits(meters);

        assertEquals(sensorCounts, calculatedCounts);
    }

    @Test
    public void testVelocityRoundTrip() {

        double distanceTraveled = 2;
        double secondsElapsed = 4;

        double velocityMetersPerSecond = distanceTraveled / secondsElapsed;


        int sensorCountsPer100ms = CTREConvert.velocityToNativeUnits(velocityMetersPerSecond);
        double calculatedVelocity = CTREConvert.nativeUnitsToVelocityMetersPerSecond(sensorCountsPer100ms);

        assertEquals(velocityMetersPerSecond, calculatedVelocity, 0.01);

    }

}
