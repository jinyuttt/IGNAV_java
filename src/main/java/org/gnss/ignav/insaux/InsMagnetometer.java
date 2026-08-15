package org.gnss.ignav.insaux;

import org.gnss.ignav.common.GeoMag;
import org.gnss.ignav.common.InsMath;
import org.gnss.ignav.constants.IgnavConstants;
import org.gnss.ignav.data.BField;
import org.gnss.ignav.data.BFieldModel;
import org.gnss.ignav.data.GTime;
import org.gnss.ignav.data.Magd;
import org.gnss.ignav.data.MagOpt;
import org.gnss.ignav.data.InsOpt;
import org.gnss.ignav.data.InsState;
import org.gnss.ignav.ins.InsAlignMech;
import org.gnss.ignav.insgnss.InsGnssState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsMagnetometer {

    private static final Logger logger = LoggerFactory.getLogger(InsMagnetometer.class);

    private static final double VAR_MAG = 5.0 * IgnavConstants.D2R;

    private InsMagnetometer() {}

    public static int magmodel(String file) {
        logger.debug("magmodel, file={}", file);
        return GeoMag.magmodel(file);
    }

    private static void untilt(Magd data, double[] Cbn, double[] mh) {
        double r = -Math.atan(Cbn[2] / Math.sqrt(1.0 - InsMath.SQR(Cbn[2])));
        double p = Math.atan2(Cbn[5], Cbn[8]);
        double[] Cbh = new double[6];

        Cbh[0] = Math.cos(p);
        Cbh[2] = Math.sin(r) * Math.sin(p);
        Cbh[4] = Math.cos(r) * Math.sin(p);
        Cbh[1] = 0.0;
        Cbh[3] = Math.cos(r);
        Cbh[5] = -Math.sin(r);
        InsMath.matmul("NN", 2, 1, 3, 1.0, Cbh, data.val, 0.0, mh);
    }

    private static double undecli(InsState ins, double[] pos) {
        double[] ep = new double[6];
        GTime.time2epoch(ins.time, ep);
        BField mag = GeoMag.getMag();
        BFieldModel model = GeoMag.getMagModel();
        GeoMag.getFieldComponents(mag, model, pos[2], GeoMag.UNITS_METERS,
                GeoMag.COORDSYS_GEODETIC, pos[0] * IgnavConstants.R2D, pos[1] * IgnavConstants.R2D,
                GeoMag.julday((int) ep[1], (int) ep[2], (int) ep[0]));
        return mag.d;
    }

    private static void undistortmag(MagOpt opt, double[] mh) {
        mh[0] = opt.sx * mh[0] + opt.ox;
        mh[1] = opt.sy * mh[1] + opt.oy;
    }

    public static double maghead(MagOpt opt, Magd data, double[] Cbn, double[] pos, InsState ins) {
        double[] mh = new double[2];
        double decli, yaw;

        logger.debug("magcalib:");

        untilt(data, Cbn, mh);
        undistortmag(opt, mh);

        decli = undecli(ins, pos);

        yaw = Math.atan2(mh[1], mh[0]);
        return yaw - decli;
    }

    private static void jacobHead2att(double[] Cbe, double[] pos, double[] dhdatt) {
        double[] Cne = new double[9];
        double[] Cbn = new double[9];

        InsMath.ned2xyz(pos, Cne);
        InsMath.matmul("TN", 3, 3, 3, 1.0, Cne, Cbe, 0.0, Cbn);

        dhdatt[0] = (Cbn[0] * (Cne[5] * Cbe[1] - Cne[4] * Cbe[2]) - Cbn[1] * (Cne[2] * Cbe[1] - Cne[1] * Cbe[2])) /
                (InsMath.SQR(Cbn[0]) + InsMath.SQR(Cbn[1]));

        dhdatt[1] = (Cbn[0] * (-Cne[5] * Cbe[0] + Cne[3] * Cbe[2]) - Cbn[1] * (-Cne[2] * Cbe[1] + Cne[0] * Cbe[2])) /
                (InsMath.SQR(Cbn[0]) + InsMath.SQR(Cbn[1]));

        dhdatt[2] = (Cbn[0] * (Cne[4] * Cbe[0] - Cne[3] * Cbe[1]) - Cbn[1] * (Cne[1] * Cbe[0] - Cne[0] * Cbe[1])) /
                (InsMath.SQR(Cbn[0]) + InsMath.SQR(Cbn[1]));
    }

    private static int magfilt(InsState ins, InsOpt opt, Magd data) {
        int nx = ins.nx;
        double[] H = new double[1 * nx];
        double[] x = ins.x;
        double[] P = ins.P;
        double R;
        double v;
        double[] Cbn = new double[9];
        double[] pos = new double[3];
        double[] Cne = new double[9];
        double magh, yaw;

        logger.debug("magfilt: nx={}", nx);

        InsMath.ecef2pos(ins.re, pos);
        InsMath.ned2xyz(pos, Cne);
        InsMath.matmul("TN", 3, 3, 3, 1.0, Cne, ins.Cbe, 0.0, Cbn);

        magh = maghead(opt.magopt, data, Cbn, pos, ins);
        yaw = Math.atan2(Cbn[1], Cbn[0]) * IgnavConstants.D2R;

        jacobHead2att(ins.Cbe, pos, H);

        v = normang(magh) - normang(yaw);
        v *= IgnavConstants.D2R;
        R = InsMath.SQR(VAR_MAG);

        double[] vArr = new double[]{v};
        double[] RArr = new double[]{R};

        if (InsAlignMech.filter(x, P, H, vArr, RArr, nx, 1) != 0) {
            logger.warn("filter error");
            return 0;
        }

        InsNhc.clp(ins, opt, x);
        return 1;
    }

    public static int magnetometer(InsState ins, InsOpt opt, Magd data) {
        logger.debug("magnetometer:");

        if (InsMath.norm(data.val, 3) <= 0.0) {
            logger.warn("no valid magnetometer data");
            return 0;
        }
        if (magfilt(ins, opt, data) == 0) {
            logger.warn("magnetic head update fail");
            return 0;
        }
        ins.stat = IgnavConstants.INSS_MAGH;
        return 1;
    }

    private static double normang(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}