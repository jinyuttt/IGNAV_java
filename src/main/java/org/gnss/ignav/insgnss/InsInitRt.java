package org.gnss.ignav.insgnss;

import org.gnss.ignav.common.InsMath;
import org.gnss.ignav.constants.IgnavConstants;
import org.gnss.ignav.data.GTime;
import org.gnss.ignav.data.Imud;
import org.gnss.ignav.data.InsOpt;
import org.gnss.ignav.data.InsState;
import org.gnss.ignav.ins.InsMech;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsInitRt {

    private static final Logger logger = LoggerFactory.getLogger(InsInitRt.class);

    private static final double MINVEL = 5.0;
    private static final double MAXGYRO = 30.0 * IgnavConstants.D2R;
    private static final double MAXDIFF = 10.0;

    private InsInitRt() {}

    public static int insinirt(double[] pos, double[] vel, GTime time, Imud imu, InsOpt opt, InsState ins) {
        double[] vr = new double[3];

        ins.stat = IgnavConstants.INSS_INIT;

        if (pos == null || vel == null) {
            logger.warn("no position/velocity data to initial");
            return 0;
        }

        InsMath.matcpy(vr, vel, 1, 3);
        if (InsMath.norm(vr, 3) == 0.0) {
            logger.warn("velocity is zero");
            return 0;
        }
        if (InsMath.norm(imu.gyro, 3) > MAXGYRO || InsMath.norm(vr, 3) < MINVEL) {
            logger.warn("gyro too large or velocity too small");
            return 0;
        }

        if (InsGnssTc.ant2inins(time, pos, vel, opt, imu, ins) == 0) {
            logger.warn("ant2inins fail");
            return 0;
        }
        ins.time = new GTime(time);

        InsMech.updateInsStateN(ins);

        logger.info("initial ins state ok");
        return 1;
    }

    public static int insinidualant(double[] pos, double[] vel, double[] rpy,
                                     GTime time, Imud imu, InsOpt opt, InsState ins) {
        double[] Cne = new double[9], Cvn = new double[9];
        double[] Ry = new double[9], Rz = new double[9];
        double[] posllh = new double[3];

        ins.stat = IgnavConstants.INSS_INIT;

        if (pos == null || vel == null || rpy == null) {
            logger.warn("no position/velocity/attitude data to initial");
            return 0;
        }

        InsMath.ecef2pos(pos, posllh);
        InsMath.ned2xyz(posllh, Cne);

        double[] RyArr = new double[9], RzArr = new double[9];
        InsMath.Ry(-rpy[1], RyArr);
        InsMath.Rz(-rpy[2], RzArr);
        InsMath.matmul("NN", 3, 3, 3, 1.0, RzArr, RyArr, 0.0, Cvn);

        InsMath.matmul33("NNT", Cne, Cvn, ins.Cvb, 3, 3, 3, 3, ins.Cbe);
        InsMath.gapv2ipv(pos, vel, ins.Cbe, ins.lever, imu, ins.re, ins.ve);

        InsMech.updateInsStateN(ins);

        ins.time = new GTime(time);
        logger.info("initial ins state from dual antenna ok");
        return 1;
    }
}