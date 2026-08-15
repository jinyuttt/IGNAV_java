package org.gnss.ignav.data;

public class InsSol {

    public GTime time;
    public int nx;
    public int ns;
    public int stat;
    public int gstat;

    public double[] cCbe;
    public double[] cre;
    public double[] cve;
    public double[] cae;
    public double[] cba;
    public double[] cbg;
    public double[] cMa;
    public double[] cMg;
    public double[] clever;

    public double[] pCbe;
    public double[] pre;
    public double[] pve;
    public double[] pae;
    public double[] pba;
    public double[] pbg;
    public double[] pMa;
    public double[] pMg;
    public double[] plever;

    public double[] sCbe;
    public double[] sre;
    public double[] sve;
    public double[] sae;
    public double[] sba;
    public double[] sbg;
    public double[] sMa;
    public double[] sMg;
    public double[] slever;

    public double[] Pc;
    public double[] Pp;
    public double[] Ps;
    public double[] F;

    public InsSol() {
        this.time = new GTime();
        this.nx = 0;
        this.ns = 0;
        this.stat = 0;
        this.gstat = 0;
        this.cCbe = new double[9];
        this.cre = new double[3];
        this.cve = new double[3];
        this.cae = new double[3];
        this.cba = new double[3];
        this.cbg = new double[3];
        this.cMa = new double[9];
        this.cMg = new double[9];
        this.clever = new double[3];
        this.pCbe = new double[9];
        this.pre = new double[3];
        this.pve = new double[3];
        this.pae = new double[3];
        this.pba = new double[3];
        this.pbg = new double[3];
        this.pMa = new double[9];
        this.pMg = new double[9];
        this.plever = new double[3];
        this.sCbe = new double[9];
        this.sre = new double[3];
        this.sve = new double[3];
        this.sae = new double[3];
        this.sba = new double[3];
        this.sbg = new double[3];
        this.sMa = new double[9];
        this.sMg = new double[9];
        this.slever = new double[3];
    }

    public void alloc(int nx) {
        this.nx = nx;
        this.Pc = new double[nx * nx];
        this.Pp = new double[nx * nx];
        this.Ps = new double[nx * nx];
        this.F = new double[nx * nx];
    }
}