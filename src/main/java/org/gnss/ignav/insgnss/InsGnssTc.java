package org.gnss.ignav.insgnss;

import org.gnss.ignav.common.InsMath;
import org.gnss.ignav.constants.IgnavConstants;
import org.gnss.ignav.data.GTime;
import org.gnss.ignav.data.Gmea;
import org.gnss.ignav.data.Gmeas;
import org.gnss.ignav.data.Imud;
import org.gnss.ignav.data.InsOpt;
import org.gnss.ignav.data.InsState;
import org.gnss.ignav.ins.InsMech;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsGnssTc {

    private static final Logger logger = LoggerFactory.getLogger(InsGnssTc.class);

    private static final double MAXVAR = 1E10;
    private static final int MAXSOLR = 2;
    private static final double MINVEL_TC = 3.0;
    private static final double MAXGYRO_TC = 30.0 * IgnavConstants.D2R;
    private static final double MAXDIFF_TC = 30.0;
    private static final int REBOOT_TC = 1;
    private static final int CHKNUMERIC_TC = 1;

    private InsGnssTc() {}

    private static int chkpcov(int nx, InsOpt opt, double[] P) {
        int i;
        double var = 0.0;
        for (i = InsGnssState.xiP(opt); i < InsGnssState.xiP(opt) + 3; i++)
            var += Math.sqrt(P[i + i * nx]);

        if ((var / 3) > MAXVAR) {
            if (P != null) {
                InsGnssLc.getP0(null, opt, P);
            }
        }
        return 0;
    }

    private static int chkvb(InsState ins) {
        double[] vb = new double[3];
        InsMath.matmul3v("TN", ins.Cbe, ins.ve, vb);
        return (Math.abs(vb[1]) < MINVEL_TC && Math.abs(vb[2]) < MINVEL_TC) ? 1 : 0;
    }

    public static int savegmeas(InsState ins, double[] solRr, double[] solQr, double[] solQv, GTime solTime, int solStat, Gmea gmea) {
        int i;
        Gmea meas = new Gmea();

        if (gmea != null) {
            if (ins.gmeas.addgmea(gmea) < 0) {
                logger.warn("add gps position/velocity measurement fail");
                return 0;
            }
        }
        if (solRr != null) {
            InsMath.matcpy(meas.pe, solRr, 1, 3);
            meas.t = new GTime(solTime);

            for (i = 0; i < 3; i++)
                meas.std[i] = Math.sqrt(solQr != null ? solQr[i] : 0.0);
            for (i = 3; i < 6; i++)
                meas.std[i] = Math.sqrt(solQv != null ? solQv[i - 3] : 0.0);

            if (ins.gmeas.addgmea(meas) < 0) {
                logger.warn("add gps position/velocity measurement fail");
                return 0;
            }
        }
        if (ins.gmeas.n > IgnavConstants.NPOS) {
            for (i = 0; i < IgnavConstants.NPOS; i++) {
                Gmea.copy(ins.gmeas.data[i + 1], ins.gmeas.data[i]);
            }
            ins.gmeas.n = IgnavConstants.NPOS;
        }
        return 1;
    }

    public static int rechkatt(InsState ins, Imud imu) {
        int i, j;
        double dt;
        double[] vel = new double[3 * IgnavConstants.NPOS];
        double[] llh = new double[3];
        double[] C = new double[9];
        double yaw;
        double[] vn = new double[3];
        double[] rpy = new double[3];
        double[] vb = new double[3];
        double[] pvb = new double[3];
        double[] Cbe = new double[9];

        for (i = 0; i < IgnavConstants.NPOS; i++) {
            if (ins.gmeas.data[i].stat <= IgnavConstants.SOLQ_DGPS)
                return 0;
        }

        if (ins.gmeas.n == IgnavConstants.NPOS) {
            for (i = IgnavConstants.NPOS; i >= 2; i--) {
                dt = GTime.timeDiff(ins.gmeas.data[i - 1].t, ins.gmeas.data[i - 2].t);
                if (dt > 3.0 || Math.abs(dt) <= 1E-5) {
                    continue;
                }
                for (j = 0; j < 3; j++) {
                    vel[3 * (IgnavConstants.NPOS - i) + j] =
                            (ins.gmeas.data[i - 1].pe[j] - ins.gmeas.data[i - 2].pe[j]) / dt;
                }
            }

            if (InsMath.chksdri(vel, IgnavConstants.NPOS - 1) == 0) {
                logger.info("no straight driving");
                return 0;
            }

            if (InsMath.norm(vel, 3) > 5.0 && InsMath.norm(imu.gyro, 3) < 5.0 * IgnavConstants.D2R) {
                InsMath.ecef2pos(ins.gmeas.data[IgnavConstants.NPOS - 1].pe, llh);
                InsMath.ned2xyz(llh, C);

                InsMath.matmul3v("TN", C, vel, vn);
                yaw = InsMath.normang(InsMath.vel2head(vn) * IgnavConstants.R2D);

                InsMath.getatt(ins, rpy);

                if (Math.abs(yaw - InsMath.normang(rpy[2] * IgnavConstants.R2D)) < 3.0)
                    return 0;

                rpy[2] = (InsMath.normang(rpy[2] * IgnavConstants.R2D) + yaw) / 2.0 * IgnavConstants.D2R;
                InsMath.rpy2dcm(rpy, C);
                InsMath.matt(C, 3, 3, ins.Cbn);

                InsMath.ned2xyz(llh, C);
                InsMath.matmul("NN", 3, 3, 3, 1.0, C, ins.Cbn, 0.0, Cbe);
                InsMath.matmul3v("TN", Cbe, ins.ve, vb);
                InsMath.matmul3v("TN", ins.Cbe, ins.ve, pvb);

                if (Math.abs(InsMath.norm(vb, 3) - InsMath.norm(pvb, 3)) < MINVEL_TC &&
                        (Math.abs(vb[1]) < Math.abs(pvb[1])) &&
                        (Math.abs(vb[2]) < Math.abs(pvb[2]))) {
                    InsMath.matcpy(ins.Cbe, Cbe, 3, 3);
                    logger.info("recheck attitude ok");
                    return 1;
                }
            }
        }
        logger.info("no recheck attitude");
        return 0;
    }

    public static int ant2inins(GTime time, double[] rr, double[] vr, InsOpt opt, Imud imu, InsState ins) {
        double[] llh = new double[3];
        double[] vn = new double[3];
        double[] C = new double[9];
        double[] rpy = new double[3];

        InsMath.ecef2pos(rr, llh);
        InsMath.ned2xyz(llh, C);
        InsMath.matmul3v("TN", C, vr, vn);

        InsMath.matcpy(ins.rn, llh, 1, 3);
        InsMath.matcpy(ins.vn, vn, 1, 3);

        rpy[2] = InsMath.vel2head(vn);
        InsMath.rpy2dcm(rpy, C);
        InsMath.matt(C, 3, 3, ins.Cbn);

        InsMath.ned2xyz(llh, C);
        InsMath.matmul("NN", 3, 3, 3, 1.0, C, ins.Cbn, 0.0, ins.Cbe);

        InsMath.gapv2ipv(rr, vr, ins.Cbe, opt.lever, imu, ins.re, ins.ve);
        return 1;
    }

    public static void propinss(InsGnssState st, InsOpt opt, InsState ins, double dt, double[] x, double[] P) {
        int nx = ins.nx;
        double[] phi = new double[nx * nx];
        double[] Q = new double[nx * nx];

        InsGnssLc.updstat(st, opt, ins, dt, ins.x, ins.P, phi, P, x, Q);
    }

    public static int tcigpos(InsGnssState st, InsOpt opt, Imud data, InsState ins, int upd,
                              RtkPosProvider rtkProvider) {
        int i;
        int nx = ins.nx;
        int info = 1;
        double dt;

        for (i = 0; i < 3; i++) {
            if (Double.isNaN(ins.re[i]) || Double.isNaN(ins.ve[i]) || Double.isNaN(ins.ae[i]) ||
                    Double.isInfinite(ins.re[i]) || Double.isInfinite(ins.ve[i]) || Double.isInfinite(ins.ae[i])) {
                logger.warn("check numeric error: nan or inf");
                return 0;
            }
        }

        ins.stat = IgnavConstants.INSS_NONE;
        if (InsMech.updateins(opt, ins, data) == 0) {
            logger.warn("ins mechanization update fail");
            return 0;
        }

        double[] P = new double[nx * nx];

        propinss(st, opt, ins, ins.dt, ins.x, ins.P);

        chkpcov(nx, opt, ins.P);

        if (upd == IgnavConstants.INSUPD_TIME) {
            ins.stat = IgnavConstants.INSS_TIME;
            info = 1;
        } else {
            ins.gstat = IgnavConstants.SOLQ_NONE;
            ins.ns = 0;

            if (rtkProvider != null && rtkProvider.hasObservation()) {
                dt = GTime.timeDiff(rtkProvider.getObsTime(), ins.time);

                if (Math.abs(dt) > 3.0) {
                    logger.warn("observation and imu sync error");
                    info = 0;
                }

                if (info != 0) {
                    info = rtkProvider.rtkpos();
                }
            } else {
                info = 0;
            }

            if (info != 0) {
                ins.ptct = new GTime(ins.time);
                ins.stat = ins.stat == IgnavConstants.INSS_REBOOT ? IgnavConstants.INSS_REBOOT : IgnavConstants.INSS_TCUD;

                logger.info("tightly couple ok");

                if (ins.ns < 4) {
                    ins.stat = IgnavConstants.INSS_LACK;
                }

                if (rtkProvider != null) {
                    savegmeas(ins, rtkProvider.getSolRr(), rtkProvider.getSolQr(),
                            rtkProvider.getSolQv(), rtkProvider.getSolTime(),
                            rtkProvider.getSolStat(), null);
                }

                rechkatt(ins, data);

                InsMech.updateInsStateN(ins);
            } else {
                logger.warn("tightly coupled fail");
                info = 0;
            }
        }

        return info;
    }

    public static int tcigposSimple(InsGnssState st, InsOpt opt, Imud data, InsState ins, int upd) {
        return tcigpos(st, opt, data, ins, upd, null);
    }

    public interface RtkPosProvider {
        boolean hasObservation();
        GTime getObsTime();
        int rtkpos();
        double[] getSolRr();
        double[] getSolQr();
        double[] getSolQv();
        GTime getSolTime();
        int getSolStat();
    }
}