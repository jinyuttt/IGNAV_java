package org.gnss.ignav.insgnss;

import org.gnss.ignav.common.InsMath;
import org.gnss.ignav.constants.IgnavConstants;
import org.gnss.ignav.data.GTime;
import org.gnss.ignav.data.InsOpt;
import org.gnss.ignav.data.InsSol;
import org.gnss.ignav.data.InsState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsRts {

    private static final Logger logger = LoggerFactory.getLogger(InsRts.class);

    private static final double MAXTIMEDIFF = 3.0;
    private static final double RTS_FACTOR = 0.9999;
    private static final double VAL_FACTOR = 10.0;

    private InsRts() {}

    private static void errCorr(InsSol pre, InsSol cur, InsOpt opt, double[] x, double[] dP) {
        double[] omg = new double[3];
        double[] dC = new double[9];
        int i;
        int iba = InsGnssState.xiBa(opt);
        int nba = InsGnssState.xnBa(opt);
        int ibg = InsGnssState.xiBg(opt);
        int nbg = InsGnssState.xnBg(opt);
        int isg = InsGnssState.xiSg(opt);
        int nsg = InsGnssState.xnSg(opt);
        int isa = InsGnssState.xiSa(opt);
        int nsa = InsGnssState.xnSa(opt);
        int ila = InsGnssState.xiLa(opt);
        int nla = InsGnssState.xnLa(opt);

        InsMath.matmul("TN", 3, 3, 3, 1.0, cur.pCbe, pre.sCbe, 0.0, dC);
        InsMath.so3Log(dC, omg, null);
        for (i = 0; i < 3; i++) omg[i] = -omg[i];

        InsMath.matcpy(x, omg, 1, 3);

        if (nsg > 0) {
            for (i = isg; i < isg + nsg; i++) {
                x[i] = pre.sMg[i - isg + (i - isg) * 3] - cur.pMg[i - isg + (i - isg) * 3];
            }
        }
        if (nsa > 0) {
            for (i = isa; i < isa + nsa; i++) {
                x[i] = pre.sMa[i - isa + (i - isa) * 3] - cur.pMa[i - isa + (i - isa) * 3];
            }
        }

        int IV = InsGnssState.xiV(opt);
        int IP = InsGnssState.xiP(opt);

        x[IV + 0] = -pre.sve[0] + cur.pve[0];
        x[IV + 1] = -pre.sve[1] + cur.pve[1];
        x[IV + 2] = -pre.sve[2] + cur.pve[2];

        x[IP + 0] = -pre.sre[0] + cur.pre[0];
        x[IP + 1] = -pre.sre[1] + cur.pre[1];
        x[IP + 2] = -pre.sre[2] + cur.pre[2];

        if (nba > 0) {
            x[iba + 0] = pre.sba[0] - cur.pba[0];
            x[iba + 1] = pre.sba[1] - cur.pba[1];
            x[iba + 2] = pre.sba[2] - cur.pba[2];
        }
        if (nbg > 0) {
            x[ibg + 0] = pre.sbg[0] - cur.pbg[0];
            x[ibg + 1] = pre.sbg[1] - cur.pbg[1];
            x[ibg + 2] = pre.sbg[2] - cur.pbg[2];
        }
        if (nla > 0) {
            for (i = ila; i < ila + nla; i++) {
                x[i] = pre.slever[i - ila] - cur.plever[i - ila];
            }
        }

        for (i = 0; i < cur.nx * cur.nx; i++) {
            dP[i] = pre.Ps[i] - cur.Pc[i];
        }
    }

    private static int valsmth(double[] Ps, double[] Pc, double[] dx, int nx) {
        double[] var = new double[3];
        int i, flag = 0;

        for (i = 6; i < 9; i++) var[i - 6] = Ps[i + i * nx] + Pc[i + i * nx];
        for (i = 6; i < 9; i++) {
            if (InsMath.SQR(dx[i]) <= InsMath.SQR(VAL_FACTOR) * var[i - 6]) continue;
            flag = 3;
        }
        for (i = 3; i < 6; i++) var[i - 3] = Ps[i + i * nx] + Pc[i + i * nx];
        for (i = 3; i < 6; i++) {
            if (InsMath.SQR(dx[i]) <= InsMath.SQR(VAL_FACTOR) * var[i - 3]) continue;
            flag++;
        }
        for (i = 0; i < 3; i++) var[i] = Ps[i + i * nx] + Pc[i + i * nx];
        for (i = 0; i < 3; i++) {
            if (InsMath.SQR(dx[i]) <= InsMath.SQR(VAL_FACTOR) * var[i]) continue;
            flag++;
        }
        return flag < 3 ? 1 : 0;
    }

    private static void lcclpRts(double[] x, InsSol cur, InsOpt opt) {
        int IV = InsGnssState.xiV(opt);
        int IP = InsGnssState.xiP(opt);
        int iba = InsGnssState.xiBa(opt);
        int nba = InsGnssState.xnBa(opt);
        int ibg = InsGnssState.xiBg(opt);
        int nbg = InsGnssState.xnBg(opt);

        if (x[0] != 0.0) {
            InsMath.corratt(x, cur.cCbe);
        }
        InsMath.matcpy(cur.sCbe, cur.cCbe, 3, 3);

        cur.sve[0] = cur.cve[0] - x[IV + 0];
        cur.sve[1] = cur.cve[1] - x[IV + 1];
        cur.sve[2] = cur.cve[2] - x[IV + 2];

        cur.sre[0] = cur.cre[0] - x[IP + 0];
        cur.sre[1] = cur.cre[1] - x[IP + 1];
        cur.sre[2] = cur.cre[2] - x[IP + 2];

        if (nba > 0 && x[iba] != 0.0) {
            cur.sba[0] = cur.cba[0] + x[iba + 0];
            cur.sba[1] = cur.cba[1] + x[iba + 1];
            cur.sba[2] = cur.cba[2] + x[iba + 2];
        } else {
            InsMath.matcpy(cur.sba, cur.cba, 1, 3);
        }
        if (nbg > 0 && x[ibg] != 0.0) {
            cur.sbg[0] = cur.cbg[0] + x[ibg + 0];
            cur.sbg[1] = cur.cbg[1] + x[ibg + 1];
            cur.sbg[2] = cur.cbg[2] + x[ibg + 2];
        } else {
            InsMath.matcpy(cur.sbg, cur.cbg, 1, 3);
        }

        InsMath.matcpy(cur.sMa, cur.cMa, 3, 3);
        InsMath.matcpy(cur.sMg, cur.cMg, 3, 3);
        InsMath.matcpy(cur.sae, cur.cae, 3, 1);
        InsMath.matcpy(cur.slever, cur.clever, 3, 1);
    }

    private static void updInsState(InsSol cur, InsState ins) {
        InsMath.matcpy(ins.lever, cur.slever, 1, 3);
        InsMath.matcpy(ins.P, cur.Ps, cur.nx, cur.nx);
        InsMath.matcpy(ins.Cbe, cur.sCbe, 3, 3);
        InsMath.matcpy(ins.re, cur.sre, 3, 1);
        InsMath.matcpy(ins.ve, cur.sve, 3, 1);
        InsMath.matcpy(ins.ae, cur.sae, 3, 1);
        InsMath.matcpy(ins.ba, cur.sba, 3, 1);
        InsMath.matcpy(ins.bg, cur.sbg, 3, 1);
        InsMath.matcpy(ins.Ma, cur.sMa, 3, 3);
        InsMath.matcpy(ins.Mg, cur.sMg, 3, 3);
    }

    public static int updInsRts(InsSol pre, InsSol cur, InsState ins, InsOpt opt) {
        int nx = cur.nx;
        int i;

        double[] dP = new double[nx * nx];
        double[] Ak = new double[nx * nx];
        double[] Pk1 = new double[nx * nx];
        double[] dx = new double[nx];
        double[] xs = new double[nx];
        double[] Ps = new double[nx * nx];

        errCorr(pre, cur, opt, dx, dP);

        InsMath.matcpy(Pk1, cur.Pp, nx, nx);
        if (InsMath.matinv(Pk1, nx) == 0) {
            InsMath.matmul33("NTN", cur.Pc, cur.F, Pk1, nx, nx, nx, nx, Ak);
        } else {
            return 0;
        }
        InsMath.matmul("NN", nx, 1, nx, RTS_FACTOR, Ak, dx, 0.0, xs);
        InsMath.matmul33("NNT", Ak, dP, Ak, nx, nx, nx, nx, Pk1);

        for (i = 0; i < nx * nx; i++) Ps[i] = Pk1[i] + cur.Pc[i];
        InsMath.matcpy(cur.Ps, Ps, nx, nx);

        if (valsmth(Ps, cur.Pc, xs, nx) != 0) {
            InsMath.matcpy(cur.Ps, Ps, nx, nx);
        } else {
            InsMath.matcpy(cur.Ps, cur.Pc, nx, nx);
            for (i = 0; i < nx; i++) xs[i] = 0.0;
        }

        lcclpRts(xs, cur, opt);
        updInsState(cur, ins);

        ins.time = new GTime(cur.time);
        ins.ns = cur.ns;
        ins.gstat = cur.gstat;
        ins.stat = IgnavConstants.INSS_RTS;

        return 1;
    }

    public static void initBcksmh(InsSol cur) {
        InsMath.matcpy(cur.sCbe, cur.cCbe, 3, 3);
        InsMath.matcpy(cur.sre, cur.cre, 3, 1);
        InsMath.matcpy(cur.sve, cur.cve, 3, 1);
        InsMath.matcpy(cur.sae, cur.cae, 3, 1);
        InsMath.matcpy(cur.sba, cur.cba, 3, 1);
        InsMath.matcpy(cur.sbg, cur.cbg, 3, 1);
        InsMath.matcpy(cur.sMa, cur.cMa, 3, 3);
        InsMath.matcpy(cur.sMg, cur.cMg, 3, 3);
        InsMath.matcpy(cur.Ps, cur.Pc, cur.nx, cur.nx);
        InsMath.matcpy(cur.slever, cur.clever, 3, 1);
    }
}