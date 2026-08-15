package org.gnss.ignav.insgnss;

import org.gnss.ignav.common.InsMath;
import org.gnss.ignav.constants.IgnavConstants;
import org.gnss.ignav.data.GTime;
import org.gnss.ignav.data.Gmea;
import org.gnss.ignav.data.Gmeas;
import org.gnss.ignav.data.Imud;
import org.gnss.ignav.data.InsOpt;
import org.gnss.ignav.data.InsState;
import org.gnss.ignav.ins.InsAlignMech;
import org.gnss.ignav.ins.InsMech;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsGnssLc {

    private static final Logger logger = LoggerFactory.getLogger(InsGnssLc.class);

    private static final int NMP = 3;
    private static final int NMV = 3;
    private static final int NM = NMP + NMV;
    private static final int IMP = 0;
    private static final int IMV = NMP;

    private static final double STD_POS = 2.5;
    private static final double STD_VEL = 0.1;
    private static final double MAXINOP = 1000.0;
    private static final double MAXINOV = 100.0;
    private static final double MAXSYNDIFF = 1.0;
    private static final double MAXUPDTIMEINT = 60.0;

    private static final double UNC_ATT = IgnavConstants.D2R * 100.0;
    private static final double UNC_VEL = 1.0;
    private static final double UNC_POS = 10.0;
    private static final double UNC_BA = 1E4 * IgnavConstants.MG2M;
    private static final double UNC_BG = 1E4 * IgnavConstants.DEG2R;
    private static final double UNC_DT = 0.01;
    private static final double UNC_SG = 1E6;
    private static final double UNC_SA = 1E6;
    private static final double UNC_RG = 1E6;
    private static final double UNC_RA = 1E6;
    private static final double UNC_LEVER = 0.5;
    private static final double UNC_OS = 0.1;
    private static final double UNC_OA = IgnavConstants.D2R * 30.0;
    private static final double UNC_CLK = 100.0;
    private static final double UNC_CLKR = 10.0;

    private InsGnssLc() {}

    private static void sysQ(int is, int n, int nx, double v, double dt, double[] Q) {
        for (int i = is; i < is + n; i++)
            Q[i + i * nx] = v * Math.abs(dt);
    }

    public static void getQ(InsGnssState st, InsOpt opt, double dt, double[] Q) {
        int nx = InsGnssState.xnX(opt);
        InsMath.setzero(Q, nx, nx);
        sysQ(st.IA, st.NA, nx, opt.psd.gyro, dt, Q);
        sysQ(st.IV, st.NV, nx, opt.psd.accl, dt, Q);
        sysQ(st.iba, st.nba, nx, opt.psd.ba, dt, Q);
        sysQ(st.ibg, st.nbg, nx, opt.psd.bg, dt, Q);
        sysQ(st.idt, st.ndt, nx, opt.psd.dt, dt, Q);
        sysQ(st.isg, st.nsg, nx, opt.psd.sg, dt, Q);
        sysQ(st.isa, st.nsa, nx, opt.psd.sa, dt, Q);
        sysQ(st.irg, st.nrg, nx, opt.psd.rg, dt, Q);
        sysQ(st.ira, st.nra, nx, opt.psd.ra, dt, Q);
        sysQ(st.iso, st.nos, nx, opt.psd.os, dt, Q);
        sysQ(st.iol, st.nol, nx, opt.psd.ol, dt, Q);
        sysQ(st.ioa, st.noa, nx, opt.psd.oa, dt, Q);
        sysQ(st.irc, st.nrc, nx, opt.psd.clk, dt, Q);
        sysQ(st.irr, st.nrr, nx, opt.psd.clkr, dt, Q);
    }

    public static void initP(int is, int ni, int nx, double unc, double unc0, double[] P0) {
        for (int i = is; i < is + ni; i++) {
            for (int j = 0; j < nx; j++) {
                if (j == i)
                    P0[j + i * nx] = InsMath.SQR(unc == 0.0 ? unc0 : unc);
                else
                    P0[j + i * nx] = P0[i + j * nx] = 0.0;
            }
        }
    }

    public static void getP0(InsGnssState st, InsOpt opt, double[] P0) {
        int nx = InsGnssState.xnX(opt);
        InsMath.setzero(P0, nx, nx);
        initP(st.IA, st.NA, nx, opt.unc.att, UNC_ATT, P0);
        initP(st.IV, st.NV, nx, opt.unc.vel, UNC_VEL, P0);
        initP(st.IP, st.NP, nx, opt.unc.pos, UNC_POS, P0);
        initP(st.iba, st.nba, nx, opt.unc.ba, UNC_BA, P0);
        initP(st.ibg, st.nbg, nx, opt.unc.bg, UNC_BG, P0);
        initP(st.idt, st.ndt, nx, opt.unc.dt, UNC_DT, P0);
        initP(st.isg, st.nsg, nx, opt.unc.sg, UNC_SG, P0);
        initP(st.isa, st.nsa, nx, opt.unc.sa, UNC_SA, P0);
        initP(st.irg, st.nrg, nx, opt.unc.rg, UNC_RG, P0);
        initP(st.ira, st.nra, nx, opt.unc.ra, UNC_RA, P0);
        initP(st.ila, st.nla, nx, opt.unc.lever, UNC_LEVER, P0);
        initP(st.iso, st.nos, nx, opt.unc.os, UNC_OS, P0);
        initP(st.ioa, st.noa, nx, opt.unc.oa, UNC_OA, P0);
        initP(st.iol, st.nol, nx, opt.unc.lever, UNC_LEVER, P0);
        initP(st.irc, st.nrc, nx, opt.unc.rc, UNC_CLK, P0);
        initP(st.irr, st.nrr, nx, opt.unc.rr, UNC_CLKR, P0);
    }

    public static void initlc(InsGnssState st, InsOpt opt, InsState ins) {
        int nx = InsGnssState.xnX(opt);
        ins.nx = nx;
        ins.nb = InsGnssState.xnRx(opt);
        ins.x = new double[nx];
        ins.P = new double[nx * nx];
        ins.xa = new double[nx];
        ins.Pa = new double[nx * nx];
        ins.xb = new double[ins.nb];
        ins.Pb = new double[ins.nb * ins.nb];
        ins.F = InsMath.eye(nx);
        ins.P0 = new double[nx * nx];

        ins.ptime = new GTime();
        ins.ptct = new GTime();
        ins.plct = new GTime();
        ins.dtrr = 0.0;
        ins.gstat = 0;
        ins.ns = 0;

        for (int i = 0; i < nx; i++)
            ins.x[i] = 0.0;
        for (int i = 0; i < nx; i++)
            ins.xa[i] = 0.0;
        for (int i = 0; i < 6; i++)
            ins.dtr[i] = 0.0;

        getP0(st, opt, ins.P);
        getP0(st, opt, ins.Pa);

        InsMath.matcpy(ins.lever, opt.lever, 3, 1);
        InsMath.matcpy(ins.ba, opt.imuerr.ba, 1, 3);
        InsMath.matcpy(ins.bg, opt.imuerr.bg, 1, 3);
        InsMath.matcpy(ins.Ma, opt.imuerr.Ma, 3, 3);
        InsMath.matcpy(ins.Mg, opt.imuerr.Mg, 3, 3);
        InsMath.matcpy(ins.Gg, opt.imuerr.Gg, 3, 3);
    }

    public static void freelc(InsState ins) {
        ins.x = null;
        ins.P = null;
        ins.xa = null;
        ins.Pa = null;
        ins.xb = null;
        ins.Pb = null;
        ins.P0 = null;
        ins.F = null;
    }

    public static void propP(InsGnssState st, InsOpt opt, double[] Q, double[] phi, double[] P0, double[] P) {
        int nx = InsGnssState.xnX(opt);
        double[] phiT = new double[nx * nx];
        double[] tmp = new double[nx * nx];
        InsMath.matt(phi, nx, nx, phiT);
        InsMath.matmul("NN", nx, nx, nx, 1.0, phi, P0, 0.0, tmp);
        InsMath.matmul("NN", nx, nx, nx, 1.0, tmp, phiT, 0.0, P);
        for (int i = 0; i < nx * nx; i++)
            P[i] += Q[i];
    }

    public static void propx(InsGnssState st, InsOpt opt, double[] x0, double[] x) {
        int nx = InsGnssState.xnX(opt);
        InsMath.matcpy(x, x0, nx, 1);
    }

    public static void getPhi1(InsGnssState st, InsOpt opt, double dt, double[] Cbe, double[] pos, double[] omgb, double[] fib, double[] phi) {
        int nx = InsGnssState.xnX(opt);
        double[] F = new double[nx * nx];
        double[] I = InsMath.eye(nx);
        getF(st, opt, Cbe, pos, omgb, fib, F);
        for (int i = 0; i < nx * nx; i++)
            F[i] *= dt;
        for (int i = 0; i < nx * nx; i++)
            phi[i] = I[i] + F[i];
    }

    public static void getF(InsGnssState st, InsOpt opt, double[] Cbe, double[] pos, double[] omgb, double[] fib, double[] F) {
        int nx = InsGnssState.xnX(opt);
        double[] I3 = InsMath.eye(3);

        InsMath.setzero(F, nx, nx);

        double[] Fv = new double[9];
        InsMath.skewsym3(fib, Fv);
        for (int i = 0; i < 9; i++)
            Fv[i] = -Fv[i];
        InsAlignMech.asiBlkMat(F, nx, nx, Fv, 3, 3, st.IA, st.IV);

        InsAlignMech.asiBlkMat(F, nx, nx, I3, 3, 3, st.IV, st.IP);

        if (st.nba > 0)
            InsAlignMech.asiBlkMat(F, nx, nx, Cbe, 3, 3, st.iba, st.IV);
        if (st.nbg > 0)
            InsAlignMech.asiBlkMat(F, nx, nx, Cbe, 3, 3, st.ibg, st.IA);
    }

    public static void updstat(InsGnssState st, InsOpt opt, InsState ins, double dt, double[] x0, double[] P0,
                               double[] phi, double[] P, double[] x, double[] Q) {
        int nx = InsGnssState.xnX(opt);

        getQ(st, opt, dt, Q);

        if (opt.exphi != 0) {
            precPhi(st, opt, dt, ins.Cbe, ins.rn, ins.omgb, ins.fb, phi);
        } else {
            getPhi1(st, opt, dt, ins.Cbe, ins.rn, ins.omgb, ins.fb, phi);
        }

        if (Math.abs(dt) >= MAXUPDTIMEINT) {
            getP0(st, opt, P);
        } else {
            propP(st, opt, Q, phi, P0, P);
        }

        propx(st, opt, x0, x);

        if (ins.P0 != null)
            InsMath.matcpy(ins.P0, P, nx, nx);
        if (ins.F != null)
            InsMath.matcpy(ins.F, phi, nx, nx);
    }

    private static void precPhi(InsGnssState st, InsOpt opt, double dt, double[] Cbe, double[] pos, double[] omgb, double[] fib, double[] Phi) {
        int nx = InsGnssState.xnX(opt);
        double[] F = new double[nx * nx];
        double[] I = InsMath.eye(nx);
        getF(st, opt, Cbe, pos, omgb, fib, F);
        for (int i = 0; i < nx * nx; i++)
            F[i] *= dt;
        expmat(F, nx, Phi);
    }

    private static void expmat(double[] A, int n, double[] E) {
        double[] I = InsMath.eye(n);
        double[] S = new double[n * n];
        double[] T = new double[n * n];
        InsMath.matcpy(S, A, n, n);
        InsMath.matcpy(E, I, n, n);
        for (int k = 1; k <= 12; k++) {
            double scale = 1.0 / k;
            InsMath.matmul("NN", n, n, n, scale, A, S, 0.0, T);
            double[] tmp = S;
            S = T;
            T = tmp;
            for (int i = 0; i < n * n; i++)
                E[i] += S[i];
        }
    }

    private static void rmlever(double[] re, double[] ve, double[] lever, double[] Cbe, double[] omgb, double[] reo, double[] veo) {
        double[] cl = new double[3], wl = new double[9], tmp = new double[3];
        InsMath.matmul3v("N", Cbe, lever, cl);
        if (re != null) {
            for (int i = 0; i < 3; i++)
                reo[i] = re[i] + cl[i];
        }
        InsMath.skewsym3(omgb, wl);
        InsMath.matmul3v("N", wl, lever, tmp);
        InsMath.matmul3v("N", Cbe, tmp, veo);
        if (ve != null) {
            for (int i = 0; i < 3; i++)
                veo[i] = ve[i] + veo[i];
        }
    }

    private static void jacobianPAtt(double[] Cbe, double[] lever, double[] dpdatt) {
        double[] cl = new double[3];
        InsMath.matmul3v("N", Cbe, lever, cl);
        InsMath.skewsym3(cl, dpdatt);
    }

    private static void jacobianPDt(double[] omgb, double[] lever, double[] Cbe, double[] ve, double[] dpddt) {
        double[] wl = new double[9], cl = new double[9];
        InsMath.skewsym3(omgb, wl);
        InsMath.matmul3v("N", wl, lever, cl);
        InsMath.matmul3v("N", Cbe, cl, dpddt);
        for (int i = 0; i < 3; i++)
            dpddt[i] += ve[i];
    }

    private static void jacobianVBg(double[] Cbe, double[] lever, double[] dvdbg) {
        double[] cl = new double[9];
        InsMath.skewsym3(lever, cl);
        InsMath.matmul3("NN", Cbe, cl, dvdbg);
    }

    private static void jacobianVAtt(double[] Cbe, double[] lever, double[] omgb, double[] dvdatt) {
        double[] cl = new double[9], wl = new double[9], omgie = new double[3];

        InsMath.skewsym3(omgb, cl);
        InsMath.matmul3("NN", cl, lever, wl);
        InsMath.matmul3("NN", Cbe, wl, cl);

        for (int i = 0; i < 3; i++) {
            double s = Cbe[2 + i * 3] * lever[0] + Cbe[5 + i * 3] * lever[1] + Cbe[8 + i * 3] * lever[2];
            omgie[i] = cl[i] - IgnavConstants.OMGE * s;
        }
        InsMath.skewsym3(omgie, dvdatt);
    }

    private static void jacobianVDt(double[] omgb, double[] Cbe, double[] lever, double[] ae, double[] dvddt) {
        double[] wl = new double[9], cl = new double[9];
        InsMath.skewsym3(omgb, wl);
        InsMath.matmul33("NNN", Cbe, wl, wl, 3, 3, 3, 3, cl);
        InsMath.matmul3v("N", cl, lever, dvddt);
        for (int i = 0; i < 3; i++)
            dvddt[i] += ae[i];
    }

    private static void jacobianVDla(double[] Cbe, double[] omgb, double[] dvdla) {
        double[] T = new double[9];
        double[] omgeMat = {0.0, IgnavConstants.OMGE, 0.0, -IgnavConstants.OMGE, 0.0, 0.0, 0.0, 0.0, 0.0};
        InsMath.matmul3("NN", omgeMat, Cbe, dvdla);
        InsMath.skewsym3(omgb, T);
        InsMath.matmul("NN", 3, 3, 3, -1.0, Cbe, T, 1.0, dvdla);
    }

    private static int buildHVR(InsGnssState st, InsOpt opt, double[] Cbe, double[] leverc, double[] omgb,
                                double[] fib, double[] meas, double[] re, double[] ve, double[] ae,
                                double[] std, double[] cov, double[] P, double[] H, double[] v, double[] R) {
        int i, j, nm = 0, nx = InsGnssState.xnX(opt);
        int[] ind = new int[NM];
        double[] R_ = new double[NM];
        double[] re_ = new double[3], ve_ = new double[3];
        double[] r1 = new double[9], v1 = new double[9], v5 = new double[9];
        double[] dt1 = new double[3], dt2 = new double[3], dla = new double[9];
        double[] I3 = InsMath.eye(3);

        rmlever(re, ve, leverc, Cbe, omgb, re_, ve_);

        jacobianPAtt(Cbe, leverc, r1);
        jacobianPDt(omgb, leverc, Cbe, ve, dt1);

        for (i = IMP; i < IMP + NMP; i++) {
            if (meas[i] != 0.0) {
                v[nm] = meas[i] - re_[i - IMP];
                if (Math.abs(v[nm]) >= MAXINOP) {
                    logger.warn("too large innovations for position");
                }
                if (H != null) {
                    for (j = st.IA; j < st.IA + st.NA; j++)
                        H[j + nm * nx] = r1[i - IMP + (j - st.IA) * 3];
                    for (j = st.IP; j < st.IP + st.NP; j++)
                        H[j + nm * nx] = -I3[i - IMP + (j - st.IP) * 3];
                    for (j = st.idt; j < st.idt + st.ndt; j++)
                        H[j + nm * nx] = dt1[i - IMP];
                    for (j = st.ila; j < st.ila + st.nla; j++)
                        H[j + nm * nx] = -Cbe[i - IMP + (j - st.ila) * 3];
                }
                R_[nm] = std[i] == 0.0 ? InsMath.SQR(STD_POS) : InsMath.SQR(std[i]);
                ind[nm++] = i;
            }
        }

        jacobianVBg(Cbe, leverc, v5);
        jacobianVAtt(Cbe, leverc, omgb, v1);
        jacobianVDt(omgb, Cbe, leverc, ae, dt2);
        jacobianVDla(Cbe, omgb, dla);

        for (i = IMV; i < IMV + NMV; i++) {
            if (meas[i] != 0.0) {
                v[nm] = meas[i] - ve_[i - IMV];
                if (Math.abs(v[nm]) >= MAXINOV) {
                    logger.warn("too large innovations for velocity");
                }
                if (H != null) {
                    for (j = st.IA; j < st.IA + st.NA; j++)
                        H[j + nm * nx] = v1[i - IMV + (j - st.IA) * 3];
                    for (j = st.IV; j < st.IV + st.NV; j++)
                        H[j + nm * nx] = -I3[i - IMV + (j - st.IV) * 3];
                    for (j = st.ibg; j < st.ibg + st.nbg; j++)
                        H[j + nm * nx] = v5[i - IMV + (j - st.ibg) * 3];
                    for (j = st.idt; j < st.idt + st.ndt; j++)
                        H[j + nm * nx] = dt2[i - IMV];
                    for (j = st.ila; j < st.ila + st.nla; j++)
                        H[j + nm * nx] = dla[i - IMV + (j - st.ila) * 3];
                }
                R_[nm] = std[i] == 0.0 ? InsMath.SQR(STD_VEL) : InsMath.SQR(std[i]);
                ind[nm++] = i;
            }
        }

        if (nm > 0 && R != null) {
            if (cov != null) {
                for (i = 0; i < nm; i++)
                    for (j = 0; j < nm; j++)
                        R[i + nm * j] = cov[ind[i] + NM * ind[j]];
            } else {
                for (i = 0; i < nm; i++)
                    R[i + i * nm] = R_[i];
            }
        }
        return nm;
    }

    private static void interppv(InsState ins, double dt, double[] pe, double[] ve, double[] ae, double[] Cbe) {
        for (int i = 0; i < 3; i++) {
            pe[i] = ins.re[i] + ins.ve[i] * dt;
            ve[i] = ins.ve[i] + ins.ae[i] * dt;
            ae[i] = ins.ae[i];
        }
        double[] rv = new double[3];
        for (int i = 0; i < 3; i++)
            rv[i] = ins.omgb[i] * dt;
        double[] Cnn = new double[9];
        InsMath.rov2dcm(rv, Cnn);
        InsMath.matmul("NN", 3, 3, 3, 1.0, Cnn, ins.Cbe, 0.0, Cbe);
    }

    private static void prepara(InsState ins, double[] fib, double[] omgb, double[] Mgc, double[] Mac, double[] Ggc, double[] bac, double[] bgc, double[] leverc) {
        InsMath.matcpy(fib, ins.fb, 1, 3);
        InsMath.matcpy(omgb, ins.omgb, 1, 3);
        InsMath.matcpy(bac, ins.ba, 1, 3);
        InsMath.matcpy(bgc, ins.bg, 1, 3);
        InsMath.matcpy(Mgc, ins.Mg, 3, 3);
        InsMath.matcpy(Mac, ins.Ma, 3, 3);
        InsMath.matcpy(Ggc, ins.Gg, 3, 3);
        InsMath.matcpy(leverc, ins.lever, 3, 1);
    }

    public static void lcclp(InsGnssState st, InsOpt opt, double[] x, double[] Cbe, double[] re, double[] ve,
                             double[] fib, double[] omgb, double[] Gg,
                             double[] rec, double[] vec, double[] aec, double[] bac, double[] bgc,
                             double[] Mac, double[] Mgc, double[] leverc, double[] Cbec, double[] fibc, double[] omgbc) {
        int nx = InsGnssState.xnX(opt);
        double[] dC = new double[9];

        for (int i = 0; i < 3; i++) {
            rec[i] = re[i] - x[st.IP + i];
            vec[i] = ve[i] - x[st.IV + i];
            aec[i] = 0.0;
        }

        double[] phi = new double[3];
        for (int i = 0; i < 3; i++)
            phi[i] = -x[st.IA + i];
        InsMath.rov2dcm(phi, dC);
        InsMath.matmul("NN", 3, 3, 3, 1.0, dC, Cbe, 0.0, Cbec);

        for (int i = 0; i < 3; i++) {
            fibc[i] = fib[i];
            omgbc[i] = omgb[i];
        }

        if (st.nba > 0) {
            for (int i = 0; i < 3; i++)
                bac[i] = bac[i] - x[st.iba + i];
        }
        if (st.nbg > 0) {
            for (int i = 0; i < 3; i++)
                bgc[i] = bgc[i] - x[st.ibg + i];
        }

        if (st.nla > 0) {
            for (int i = 0; i < 3; i++)
                leverc[i] = leverc[i] - x[st.ila + i];
        }
    }

    private static void updinss(InsState ins, double[] re, double[] ve, double[] Cbe, double[] fib,
                                double[] omgb, double[] ba, double[] bg, double[] Mg, double[] Ma, double[] lever) {
        InsMath.matcpy(ins.re, re, 1, 3);
        InsMath.matcpy(ins.ve, ve, 1, 3);
        InsMath.matcpy(ins.Cbe, Cbe, 3, 3);
        InsMath.matcpy(ins.fb, fib, 1, 3);
        InsMath.matcpy(ins.omgb, omgb, 1, 3);
        InsMath.matcpy(ins.ba, ba, 1, 3);
        InsMath.matcpy(ins.bg, bg, 1, 3);
        InsMath.matcpy(ins.Mg, Mg, 3, 3);
        InsMath.matcpy(ins.Ma, Ma, 3, 3);
        InsMath.matcpy(ins.lever, lever, 3, 1);
    }

    private static int valsol(double[] x, int nx, double[] P, double[] R, double[] v, int nv, double thres) {
        double fact = thres * thres;
        for (int i = 0; i < nv; i++) {
            if (Math.abs(v[i]) > fact) {
                return 0;
            }
        }
        return 1;
    }

    public static int lcfilt(InsGnssState st, InsOpt opt, InsState ins, double[] meas, double[] std, double[] cov,
                             double dt, double[] x, double[] P) {
        int nx = InsGnssState.xnX(opt);
        int nm, stat;
        double[] re = new double[3], ve = new double[3], ae = new double[3], Cbe = new double[9];
        double[] fib = new double[3], omgb = new double[3];
        double[] rec = new double[3], vec = new double[3], aec = new double[3], Cbec = new double[9];
        double[] fibc = new double[3], omgbc = new double[3];
        double[] bac = new double[3], bgc = new double[3], Mac = new double[9], Mgc = new double[9], Ggc = new double[9], leverc = new double[3];

        ins.age = dt;
        if (Math.abs(dt) >= MAXSYNDIFF) {
            logger.warn("gnss and ins time synchronization fail,dt={}", dt);
            return 0;
        }

        interppv(ins, dt, re, ve, ae, Cbe);
        prepara(ins, fib, omgb, Mgc, Mac, Ggc, bac, bgc, leverc);

        double[] H = new double[NM * nx];
        double[] v = new double[NM];
        double[] R = new double[NM * NM];

        nm = buildHVR(st, opt, Cbe, leverc, omgb, fib, meas, re, ve, ae, std, cov, P, H, v, R);

        if (nm > 0) {
            if (InsAlignMech.filter(x, P, H, v, R, nx, nm) != 0) {
                logger.warn("filter error");
                return 0;
            }
        } else {
            logger.info("no gnss position and velocity measurement");
            return 0;
        }

        lcclp(st, opt, x, Cbe, re, ve, fib, omgb, Ggc, rec, vec, aec, bac, bgc, Mac, Mgc, leverc, Cbec, fibc, omgbc);

        nm = buildHVR(st, opt, Cbec, leverc, omgbc, fibc, meas, rec, vec, aec, std, cov, P, H, v, R);
        if (nm > 0) {
            stat = valsol(x, nx, P, R, v, nm, 10.0);
            if (stat != 0) {
                updinss(ins, rec, vec, Cbec, fibc, omgbc, bac, bgc, Mgc, Mac, leverc);
            }
        } else {
            stat = 0;
        }

        return stat;
    }

    public static int lcigpos(InsGnssState st, InsOpt opt, Imud data, InsState ins, Gmeas gnss, int upd) {
        int nx = InsGnssState.xnX(opt);
        double[] phi = new double[nx * nx];
        double[] Q = new double[nx * nx];
        double[] P = new double[nx * nx];
        double[] x = new double[nx];

        updstat(st, opt, ins, ins.dt, ins.x, ins.P, phi, P, x, Q);
        InsMath.matcpy(ins.x, x, nx, 1);
        InsMath.matcpy(ins.P, P, nx, nx);

        ins.stat = IgnavConstants.INSS_NONE;
        if (InsMech.updateinsn(opt, ins, data) == 0) {
            logger.warn("ins mechanization updates fail");
            return 0;
        }

        if (upd == IgnavConstants.INSUPD_INSS)
            return 1;

        if (upd == IgnavConstants.INSUPD_TIME) {
            ins.stat = IgnavConstants.INSS_TIME;
        } else if (upd == IgnavConstants.INSUPD_MEAS && gnss != null) {
            double[] meas = new double[NM];
            double[] std = new double[NM];
            for (int i = 0; i < 3; i++) {
                meas[i + 0] = gnss.data[gnss.n - 1].pe[i];
                meas[i + 3] = gnss.data[gnss.n - 1].ve[i];
            }
            InsMath.matcpy(std, gnss.data[gnss.n - 1].std, 1, NM);

            double dtGnss = GTime.timeDiff(gnss.data[gnss.n - 1].t, ins.time);
            int stat = lcfilt(st, opt, ins, meas, std, null, dtGnss, x, P);

            if (stat != 0) {
                InsMath.matcpy(ins.x, x, nx, 1);
                InsMath.matcpy(ins.P, P, nx, nx);
                ins.stat = IgnavConstants.INSS_LCUD;
            }
            ins.plct = new GTime(ins.time);
        }

        return 1;
    }

    public static int lcigposSimple(InsGnssState st, InsOpt opt, Imud data, InsState ins,
                                     double[] gnssPos, double[] gnssVel, double[] gnssStd, GTime gnssTime) {
        Gmeas gmeas = new Gmeas();
        gmeas.alloc(1);
        gmeas.n = 1;
        InsMath.matcpy(gmeas.data[0].pe, gnssPos, 1, 3);
        InsMath.matcpy(gmeas.data[0].ve, gnssVel, 1, 3);
        InsMath.matcpy(gmeas.data[0].std, gnssStd, 1, 6);
        gmeas.data[0].t = new GTime(gnssTime);
        return lcigpos(st, opt, data, ins, gmeas, IgnavConstants.INSUPD_MEAS);
    }
}