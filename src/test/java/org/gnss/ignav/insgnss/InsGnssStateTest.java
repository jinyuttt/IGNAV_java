package org.gnss.ignav.insgnss;

import org.gnss.ignav.constants.IgnavConstants;
import org.gnss.ignav.data.InsOpt;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class InsGnssStateTest {

    private InsOpt createDefaultOpt() {
        InsOpt opt = new InsOpt();
        opt.baopt = IgnavConstants.INS_BAEST;
        opt.bgopt = IgnavConstants.INS_BGEST;
        return opt;
    }

    @Test
    void testXnX() {
        InsOpt opt = createDefaultOpt();
        int nx = InsGnssState.xnX(opt);
        assertTrue(nx > 0);
        assertEquals(15, nx);
    }

    @Test
    void testXnXMinimal() {
        InsOpt opt = new InsOpt();
        int nx = InsGnssState.xnX(opt);
        assertEquals(9, nx);
    }

    @Test
    void testXiV() {
        InsOpt opt = createDefaultOpt();
        int iv = InsGnssState.xiV(opt);
        assertEquals(3, iv);
    }

    @Test
    void testXiP() {
        InsOpt opt = createDefaultOpt();
        int ip = InsGnssState.xiP(opt);
        assertEquals(6, ip);
    }

    @Test
    void testXiBa() {
        InsOpt opt = createDefaultOpt();
        int iba = InsGnssState.xiBa(opt);
        assertEquals(9, iba);
    }

    @Test
    void testXiBg() {
        InsOpt opt = createDefaultOpt();
        int ibg = InsGnssState.xiBg(opt);
        assertEquals(12, ibg);
    }

    @Test
    void testXnBa() {
        InsOpt opt = createDefaultOpt();
        assertEquals(3, InsGnssState.xnBa(opt));
    }

    @Test
    void testXnBg() {
        InsOpt opt = createDefaultOpt();
        assertEquals(3, InsGnssState.xnBg(opt));
    }

    @Test
    void testXnA() {
        InsOpt opt = createDefaultOpt();
        assertEquals(3, InsGnssState.xnA(opt));
    }

    @Test
    void testXiA() {
        InsOpt opt = createDefaultOpt();
        assertEquals(0, InsGnssState.xiA(opt));
    }
}