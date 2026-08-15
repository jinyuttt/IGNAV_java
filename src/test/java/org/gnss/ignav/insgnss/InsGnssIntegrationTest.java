package org.gnss.ignav.insgnss;

import org.gnss.ignav.adapter.GnssPositionResult;
import org.gnss.ignav.constants.IgnavConstants;
import org.gnss.ignav.data.GTime;
import org.gnss.ignav.data.Imud;
import org.gnss.ignav.data.InsOpt;
import org.gnss.ignav.data.InsState;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class InsGnssIntegrationTest {

    private InsOpt createDefaultOpt() {
        InsOpt opt = new InsOpt();
        opt.baopt = IgnavConstants.INS_BAEST;
        opt.bgopt = IgnavConstants.INS_BGEST;
        return opt;
    }

    private GnssPositionResult createGnssResult() {
        GnssPositionResult result = new GnssPositionResult();
        result.time = new GTime(IgnavConstants.GPST0_TIME + 1000, 0.0);
        result.posEcef[0] = -2267749.234;
        result.posEcef[1] = 5009389.567;
        result.posEcef[2] = 3221290.123;
        result.velEcef[0] = 10.0;
        result.velEcef[1] = 20.0;
        result.velEcef[2] = 5.0;
        result.numSat = 12;
        result.status = GnssPositionResult.SolutionStatus.FIX;
        result.posStd[0] = 0.01;
        result.posStd[1] = 0.01;
        result.posStd[2] = 0.02;
        return result;
    }

    @Test
    void testInsGnssCreation() {
        InsOpt opt = createDefaultOpt();
        InsGnss ignav = new InsGnss(opt);

        assertNotNull(ignav.getInsState());
        assertEquals(IgnavConstants.INSS_NONE, ignav.getInsStatus());
    }

    @Test
    void testInsInitWithGnssResult() {
        InsOpt opt = createDefaultOpt();
        InsGnss ignav = new InsGnss(opt);

        GnssPositionResult gnssResult = createGnssResult();
        Imud imu = new Imud();
        imu.time = new GTime(gnssResult.time);
        imu.gyro = new double[]{0.0, 0.0, 0.0};
        imu.accl = new double[]{0.0, 0.0, 9.8};

        int ret = ignav.initIns(gnssResult, imu);
        assertTrue(ret == 0 || ret == 1);
    }

    @Test
    void testInsStateCreation() {
        InsState ins = new InsState();
        assertNotNull(ins.re);
        assertNotNull(ins.ve);
        assertNotNull(ins.Cbe);
        assertEquals(3, ins.re.length);
        assertEquals(3, ins.ve.length);
        assertEquals(9, ins.Cbe.length);
    }

    @Test
    void testInsOptDefaults() {
        InsOpt opt = new InsOpt();
        assertNotNull(opt);
    }

    @Test
    void testImudCreation() {
        Imud imu = new Imud();
        assertNotNull(imu.gyro);
        assertNotNull(imu.accl);
        assertEquals(3, imu.gyro.length);
        assertEquals(3, imu.accl.length);
    }

    @Test
    void testGTimeOperations() {
        GTime t1 = new GTime(1000, 0.5);
        GTime t2 = new GTime(1000, 0.0);

        assertEquals(0.5, GTime.timeDiff(t1, t2), 1E-12);
        assertEquals(-0.5, GTime.timeDiff(t2, t1), 1E-12);
    }

    @Test
    void testConstantsConsistency() {
        assertTrue(IgnavConstants.RE_WGS84 > 0);
        assertTrue(IgnavConstants.FE_WGS84 > 0 && IgnavConstants.FE_WGS84 < 1);
        assertTrue(IgnavConstants.OMGE > 0);
        assertTrue(IgnavConstants.GPST0_TIME > 0);
        assertTrue(IgnavConstants.D2R > 0);
        assertTrue(IgnavConstants.R2D > 0);
        assertEquals(IgnavConstants.D2R * IgnavConstants.R2D, 1.0, 1E-12);
    }
}