package org.gnss.ignav.adapter;

import org.gnss.ignav.data.Gmea;
import org.gnss.ignav.data.Gmeas;

public final class GnssResultAdapter {

    private GnssResultAdapter() {}

    public static Gmea toGmea(GnssPositionResult result) {
        Gmea gmea = new Gmea();
        if (result == null || !result.isValid()) {
            return gmea;
        }
        gmea.t = new org.gnss.ignav.data.GTime(result.time);
        gmea.ns = result.numSat;
        gmea.stat = result.status.getCode();

        System.arraycopy(result.posEcef, 0, gmea.pe, 0, 3);
        System.arraycopy(result.velEcef, 0, gmea.ve, 0, 3);

        for (int i = 0; i < 3; i++) {
            gmea.std[i] = result.posStd[i];
            gmea.std[i + 3] = result.velStd[i];
        }

        System.arraycopy(result.posCov, 0, gmea.covp, 0, 9);
        System.arraycopy(result.velCov, 0, gmea.covv, 0, 9);

        return gmea;
    }

    public static Gmeas toGmeas(GnssPositionResult[] results) {
        Gmeas gmeas = new Gmeas();
        gmeas.alloc(results.length);
        for (int i = 0; i < results.length; i++) {
            Gmea gmea = toGmea(results[i]);
            Gmea.copy(gmea, gmeas.data[i]);
        }
        gmeas.n = results.length;
        return gmeas;
    }

    public static GnssPositionResult fromGmea(Gmea gmea) {
        GnssPositionResult result = new GnssPositionResult();
        if (gmea == null) return result;

        result.time = new org.gnss.ignav.data.GTime(gmea.t);
        result.numSat = gmea.ns;
        result.status = GnssPositionResult.SolutionStatus.fromCode(gmea.stat);

        System.arraycopy(gmea.pe, 0, result.posEcef, 0, 3);
        System.arraycopy(gmea.ve, 0, result.velEcef, 0, 3);

        for (int i = 0; i < 3; i++) {
            result.posStd[i] = gmea.std[i];
            result.velStd[i] = gmea.std[i + 3];
        }

        System.arraycopy(gmea.covp, 0, result.posCov, 0, 9);
        System.arraycopy(gmea.covv, 0, result.velCov, 0, 9);

        return result;
    }
}