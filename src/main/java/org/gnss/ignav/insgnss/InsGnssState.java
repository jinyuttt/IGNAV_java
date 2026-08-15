package org.gnss.ignav.insgnss;

import org.gnss.ignav.constants.IgnavConstants;
import org.gnss.ignav.data.InsOpt;

public final class InsGnssState {

    public int IA, NA;
    public int IV, NV;
    public int IP, NP;
    public int iba, nba;
    public int ibg, nbg;
    public int idt, ndt;
    public int isa, nsa;
    public int isg, nsg;
    public int ira, nra;
    public int irg, nrg;
    public int ila, nla;
    public int iso, nos;
    public int iol, nol;
    public int ioa, noa;
    public int irc, nrc;
    public int irr, nrr;
    public int icm, ncm;
    public int ivm, nvm;

    public InsGnssState() {
        IA = NA = IV = NV = IP = NP = 0;
        iba = nba = ibg = nbg = 0;
        idt = ndt = isa = nsa = isg = nsg = 0;
        ira = nra = irg = nrg = 0;
        ila = nla = iso = nos = iol = nol = ioa = noa = 0;
        irc = nrc = irr = nrr = icm = ncm = ivm = nvm = 0;
    }

    public static int xnA(InsOpt opt) {
        return 3;
    }

    public static int xnV(InsOpt opt) {
        return 3;
    }

    public static int xnP(InsOpt opt) {
        return 3;
    }

    public static int xnBa(InsOpt opt) {
        return opt.baopt == IgnavConstants.INS_BAEST ? 3 : 0;
    }

    public static int xnBg(InsOpt opt) {
        return opt.bgopt == IgnavConstants.INS_BGEST ? 3 : 0;
    }

    public static int xnDt(InsOpt opt) {
        return opt.estdt != 0 ? 1 : 0;
    }

    public static int xnSg(InsOpt opt) {
        return opt.estsg != 0 ? 6 : 0;
    }

    public static int xnSa(InsOpt opt) {
        return opt.estsa != 0 ? 6 : 0;
    }

    public static int xnRg(InsOpt opt) {
        return opt.estrg == IgnavConstants.INS_RGEST ? 6 : 0;
    }

    public static int xnRa(InsOpt opt) {
        return opt.estra == IgnavConstants.INS_RAEST ? 6 : 0;
    }

    public static int xnLa(InsOpt opt) {
        return opt.estlever == IgnavConstants.INS_RAEST ? 3 : 0;
    }

    public static int xnOs(InsOpt opt) {
        return opt.estodos != 0 ? 1 : 0;
    }

    public static int xnOl(InsOpt opt) {
        return opt.estodol != 0 ? 3 : 0;
    }

    public static int xnOa(InsOpt opt) {
        return opt.estodoa != 0 ? 3 : 0;
    }

    public static int xnCm(InsOpt opt) {
        return 0;
    }

    public static int xnVm(InsOpt opt) {
        return 0;
    }

    public static int xnRc(InsOpt opt) {
        return 0;
    }

    public static int xnRr(InsOpt opt) {
        return 0;
    }

    public static int xnCl(InsOpt opt) {
        return xnP(opt) + xnV(opt) + xnA(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) +
                xnSg(opt) + xnSa(opt) + xnRg(opt) + xnRa(opt) + xnLa(opt) +
                xnOs(opt) + xnOl(opt) + xnOa(opt) + xnCm(opt) + xnVm(opt);
    }

    public static int xnRx(InsOpt opt) {
        return xnCl(opt) + xnRc(opt) + xnRr(opt);
    }

    public static int xnX(InsOpt opt) {
        return xnRx(opt);
    }

    public static int xiA(InsOpt opt) {
        return 0;
    }

    public static int xiV(InsOpt opt) {
        return 3;
    }

    public static int xiP(InsOpt opt) {
        return 6;
    }

    public static int xiBa(InsOpt opt) {
        return 9;
    }

    public static int xiBg(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt);
    }

    public static int xiDt(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt);
    }

    public static int xiSg(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt);
    }

    public static int xiSa(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt);
    }

    public static int xiRg(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt) + xnSa(opt);
    }

    public static int xiRa(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt) + xnSa(opt) + xnRg(opt);
    }

    public static int xiLa(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt) + xnSa(opt) +
                xnRg(opt) + xnRa(opt);
    }

    public static int xiOs(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt) + xnSa(opt) +
                xnRg(opt) + xnRa(opt) + xnLa(opt);
    }

    public static int xiOl(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt) + xnSa(opt) +
                xnRg(opt) + xnRa(opt) + xnLa(opt) + xnOs(opt);
    }

    public static int xiOa(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt) + xnSa(opt) +
                xnRg(opt) + xnRa(opt) + xnLa(opt) + xnOs(opt) + xnOl(opt);
    }

    public static int xiCm(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt) + xnSa(opt) +
                xnRg(opt) + xnRa(opt) + xnLa(opt) + xnOs(opt) + xnOl(opt) + xnOa(opt);
    }

    public static int xiVm(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt) + xnSa(opt) +
                xnRg(opt) + xnRa(opt) + xnLa(opt) + xnOs(opt) + xnOl(opt) + xnOa(opt) + xnCm(opt);
    }

    public static int xiRc(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt) + xnSa(opt) +
                xnRg(opt) + xnRa(opt) + xnLa(opt) + xnOs(opt) + xnOl(opt) + xnOa(opt) + xnCm(opt) + xnVm(opt);
    }

    public static int xiRr(InsOpt opt) {
        return xnA(opt) + xnV(opt) + xnP(opt) + xnBa(opt) + xnBg(opt) + xnDt(opt) + xnSg(opt) + xnSa(opt) +
                xnRg(opt) + xnRa(opt) + xnLa(opt) + xnOs(opt) + xnOl(opt) + xnOa(opt) + xnCm(opt) + xnVm(opt) + xnRc(opt);
    }

    public void init(InsOpt opt) {
        IA = xiA(opt);
        NA = xnA(opt);
        IV = xiV(opt);
        NV = xnV(opt);
        IP = xiP(opt);
        NP = xnP(opt);
        iba = xiBa(opt);
        nba = xnBa(opt);
        ibg = xiBg(opt);
        nbg = xnBg(opt);
        idt = xiDt(opt);
        ndt = xnDt(opt);
        isg = xiSg(opt);
        nsg = xnSg(opt);
        isa = xiSa(opt);
        nsa = xnSa(opt);
        irg = xiRg(opt);
        nrg = xnRg(opt);
        ira = xiRa(opt);
        nra = xnRa(opt);
        ila = xiLa(opt);
        nla = xnLa(opt);
        iso = xiOs(opt);
        nos = xnOs(opt);
        iol = xiOl(opt);
        nol = xnOl(opt);
        ioa = xiOa(opt);
        noa = xnOa(opt);
        irc = xiRc(opt);
        nrc = xnRc(opt);
        irr = xiRr(opt);
        nrr = xnRr(opt);
        icm = xiCm(opt);
        ncm = xnCm(opt);
        ivm = xiVm(opt);
        nvm = xnVm(opt);
    }
}