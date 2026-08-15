package org.gnss.ignav.adapter;

import org.gnss.ignav.data.GTime;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class GnssPositionResultTest {

    @Test
    void testDefaultConstructor() {
        GnssPositionResult result = new GnssPositionResult();
        assertNotNull(result.time);
        assertNotNull(result.posEcef);
        assertNotNull(result.velEcef);
        assertNotNull(result.posStd);
        assertNotNull(result.velStd);
        assertEquals(3, result.posEcef.length);
        assertEquals(3, result.velEcef.length);
        assertEquals(GnssPositionResult.SolutionStatus.NONE, result.status);
    }

    @Test
    void testIsValid() {
        GnssPositionResult result = new GnssPositionResult();
        assertFalse(result.isValid());

        result.status = GnssPositionResult.SolutionStatus.FIX;
        assertTrue(result.isValid());

        result.status = GnssPositionResult.SolutionStatus.SPP;
        assertTrue(result.isValid());
    }

    @Test
    void testIsFixed() {
        GnssPositionResult result = new GnssPositionResult();
        assertFalse(result.isFixed());

        result.status = GnssPositionResult.SolutionStatus.FIX;
        assertTrue(result.isFixed());

        result.status = GnssPositionResult.SolutionStatus.FLOAT;
        assertFalse(result.isFixed());
    }

    @Test
    void testHasVelocity() {
        GnssPositionResult result = new GnssPositionResult();
        assertFalse(result.hasVelocity());

        result.velEcef[0] = 1.0;
        assertTrue(result.hasVelocity());
    }

    @Test
    void testSolutionStatusFromCode() {
        assertEquals(GnssPositionResult.SolutionStatus.NONE, GnssPositionResult.SolutionStatus.fromCode(0));
        assertEquals(GnssPositionResult.SolutionStatus.FIX, GnssPositionResult.SolutionStatus.fromCode(1));
        assertEquals(GnssPositionResult.SolutionStatus.FLOAT, GnssPositionResult.SolutionStatus.fromCode(2));
        assertEquals(GnssPositionResult.SolutionStatus.SPP, GnssPositionResult.SolutionStatus.fromCode(3));
        assertEquals(GnssPositionResult.SolutionStatus.OTHER, GnssPositionResult.SolutionStatus.fromCode(99));
    }

    @Test
    void testGnssResultAdapterRoundTrip() {
        GnssPositionResult result = new GnssPositionResult();
        result.time = new GTime(1000, 0.5);
        result.posEcef[0] = -2267749.234;
        result.posEcef[1] = 5009389.567;
        result.posEcef[2] = 3221290.123;
        result.velEcef[0] = 10.0;
        result.velEcef[1] = 20.0;
        result.velEcef[2] = 30.0;
        result.numSat = 12;
        result.status = GnssPositionResult.SolutionStatus.FIX;
        result.posStd[0] = 0.01;
        result.posStd[1] = 0.02;
        result.posStd[2] = 0.03;

        org.gnss.ignav.data.Gmea gmea = GnssResultAdapter.toGmea(result);
        GnssPositionResult result2 = GnssResultAdapter.fromGmea(gmea);

        assertEquals(result.time.time, result2.time.time);
        assertEquals(result.time.sec, result2.time.sec, 1E-12);
        assertEquals(result.posEcef[0], result2.posEcef[0], 1E-6);
        assertEquals(result.velEcef[0], result2.velEcef[0], 1E-6);
        assertEquals(result.numSat, result2.numSat);
        assertEquals(result.status, result2.status);
        assertEquals(result.posStd[0], result2.posStd[0], 1E-6);
    }
}