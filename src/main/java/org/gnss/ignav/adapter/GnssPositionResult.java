package org.gnss.ignav.adapter;

import org.gnss.ignav.data.GTime;

public class GnssPositionResult {

    public enum SolutionStatus {
        NONE(0),
        FIX(1),
        FLOAT(2),
        SPP(3),
        DGPS(4),
        PPP(5),
        OTHER(9);

        private final int code;
        SolutionStatus(int code) { this.code = code; }
        public int getCode() { return code; }
        public static SolutionStatus fromCode(int code) {
            for (SolutionStatus s : values()) {
                if (s.code == code) return s;
            }
            return OTHER;
        }
    }

    public GTime time;

    public double[] posEcef;

    public double[] velEcef;

    public double[] posStd;

    public double[] velStd;

    public double[] posCov;

    public double[] velCov;

    public int numSat;

    public SolutionStatus status;

    public double age;

    public double ratio;

    public GnssPositionResult() {
        this.time = new GTime();
        this.posEcef = new double[3];
        this.velEcef = new double[3];
        this.posStd = new double[3];
        this.velStd = new double[3];
        this.posCov = new double[9];
        this.velCov = new double[9];
        this.numSat = 0;
        this.status = SolutionStatus.NONE;
        this.age = 0.0;
        this.ratio = 0.0;
    }

    public boolean isValid() {
        return status != SolutionStatus.NONE;
    }

    public boolean isFixed() {
        return status == SolutionStatus.FIX;
    }

    public boolean hasVelocity() {
        return velEcef[0] != 0.0 || velEcef[1] != 0.0 || velEcef[2] != 0.0;
    }
}