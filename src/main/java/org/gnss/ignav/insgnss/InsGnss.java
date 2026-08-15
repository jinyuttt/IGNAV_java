package org.gnss.ignav.insgnss;

import org.gnss.ignav.adapter.GnssObservationProvider;
import org.gnss.ignav.adapter.GnssPositionResult;
import org.gnss.ignav.adapter.GnssResultAdapter;
import org.gnss.ignav.common.InsMath;
import org.gnss.ignav.constants.IgnavConstants;
import org.gnss.ignav.data.GTime;
import org.gnss.ignav.data.Gmea;
import org.gnss.ignav.data.Gmeas;
import org.gnss.ignav.data.Imud;
import org.gnss.ignav.data.InsOpt;
import org.gnss.ignav.data.InsState;
import org.gnss.ignav.data.Odod;
import org.gnss.ignav.ins.InsMech;
import org.gnss.ignav.insaux.InsNhc;
import org.gnss.ignav.insaux.InsOdo;
import org.gnss.ignav.insaux.InsZaru;
import org.gnss.ignav.insaux.InsZvu;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class InsGnss {

    private static final Logger logger = LoggerFactory.getLogger(InsGnss.class);

    private InsState ins;
    private InsOpt opt;
    private InsGnssState st;
    private InsGnssTc.RtkPosProvider rtkProvider;

    public InsGnss(InsOpt opt) {
        this.opt = opt;
        this.ins = new InsState();
        this.st = new InsGnssState();
        this.ins.nx = InsGnssState.xnX(opt);
        int nx = this.ins.nx;
        this.ins.P = new double[nx * nx];
        this.ins.x = new double[nx];
    }

    public InsState getInsState() {
        return ins;
    }

    public void setRtkPosProvider(InsGnssTc.RtkPosProvider provider) {
        this.rtkProvider = provider;
    }

    public int initIns(double[] pos, double[] vel, GTime time, Imud imu) {
        return InsInitRt.insinirt(pos, vel, time, imu, opt, ins);
    }

    public int initInsDualAnt(double[] pos, double[] vel, double[] rpy, GTime time, Imud imu) {
        return InsInitRt.insinidualant(pos, vel, rpy, time, imu, opt, ins);
    }

    public int updateIns(Imud data) {
        return InsMech.updateins(opt, ins, data);
    }

    public int updateInsBackward(Imud data) {
        return org.gnss.ignav.ins.InsBackMech.updateinsbn(opt, ins, data);
    }

    public int lcUpdate(Imud data, Gmeas gnss, int upd) {
        return InsGnssLc.lcigpos(st, opt, data, ins, gnss, upd);
    }

    public int lcUpdateSimple(Imud data, double[] gnssPos, double[] gnssVel, double[] gnssStd, GTime gnssTime) {
        return InsGnssLc.lcigposSimple(st, opt, data, ins, gnssPos, gnssVel, gnssStd, gnssTime);
    }

    public int tcUpdate(Imud data, int upd) {
        return InsGnssTc.tcigpos(st, opt, data, ins, upd, rtkProvider);
    }

    public int zvu(Imud imu, int flag) {
        return InsZvu.zvu(ins, opt, imu, flag);
    }

    public int zaru(Imud imu, int flag) {
        return InsZaru.zaru(ins, opt, imu, flag);
    }

    public int nhc(Imud imu) {
        return InsNhc.nhc(ins, opt, imu);
    }

    public int odo(Odod odoData, Imud imu) {
        return InsOdo.odo(opt, imu, odoData, ins);
    }

    public void initOdo() {
        InsOdo.initodo(opt.odopt, ins);
    }

    public int getInsStatus() {
        return ins.stat;
    }

    public boolean isInsMech() {
        return ins.stat == IgnavConstants.INSS_MECH;
    }

    public boolean isInsLcUd() {
        return ins.stat == IgnavConstants.INSS_LCUD;
    }

    public boolean isInsTcUd() {
        return ins.stat == IgnavConstants.INSS_TCUD;
    }

    public boolean isInsZvu() {
        return ins.stat == IgnavConstants.INSS_ZVU;
    }

    public boolean isInsNhc() {
        return ins.stat == IgnavConstants.INSS_NHC;
    }

    public boolean isInsOdo() {
        return ins.stat == IgnavConstants.INSS_ODO;
    }

    public boolean isInsRts() {
        return ins.stat == IgnavConstants.INSS_RTS;
    }

    public int initIns(GnssPositionResult gnssResult, Imud imu) {
        if (gnssResult == null || !gnssResult.isValid()) {
            logger.warn("invalid gnss result for ins initialization");
            return 0;
        }
        return InsInitRt.insinirt(gnssResult.posEcef, gnssResult.velEcef, gnssResult.time, imu, opt, ins);
    }

    public int lcUpdate(Imud data, GnssPositionResult gnssResult, int upd) {
        if (gnssResult == null || !gnssResult.isValid()) {
            return 0;
        }
        Gmea gmea = GnssResultAdapter.toGmea(gnssResult);
        Gmeas gmeas = new Gmeas();
        gmeas.alloc(1);
        Gmea.copy(gmea, gmeas.data[0]);
        gmeas.n = 1;
        return InsGnssLc.lcigpos(st, opt, data, ins, gmeas, upd);
    }

    public int lcUpdate(Imud data, GnssPositionResult[] gnssResults, int upd) {
        if (gnssResults == null || gnssResults.length == 0) {
            return 0;
        }
        Gmeas gmeas = GnssResultAdapter.toGmeas(gnssResults);
        return InsGnssLc.lcigpos(st, opt, data, ins, gmeas, upd);
    }

    public GnssPositionResult getInsPositionResult() {
        GnssPositionResult result = new GnssPositionResult();
        result.time = new GTime(ins.time);
        System.arraycopy(ins.re, 0, result.posEcef, 0, 3);
        System.arraycopy(ins.ve, 0, result.velEcef, 0, 3);
        result.status = GnssPositionResult.SolutionStatus.fromCode(ins.stat);
        return result;
    }
}