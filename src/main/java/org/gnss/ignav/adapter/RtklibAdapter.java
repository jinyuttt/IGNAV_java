package org.gnss.ignav.adapter;

import org.gnss.ignav.data.GTime;

public final class RtklibAdapter {

    private RtklibAdapter() {}

    public static GnssPositionResult fromRtklibSol(Object rtklibSol) {
        GnssPositionResult result = new GnssPositionResult();
        if (rtklibSol == null) return result;

        try {
            Class<?> solClass = rtklibSol.getClass();

            Object solTime = solClass.getField("time").get(rtklibSol);
            if (solTime != null) {
                long t = solTime.getClass().getField("time").getLong(solTime);
                double s = solTime.getClass().getField("sec").getDouble(solTime);
                result.time = new GTime(t, s);
            }

            double[] rr = (double[]) solClass.getField("rr").get(rtklibSol);
            if (rr != null && rr.length >= 3) {
                System.arraycopy(rr, 0, result.posEcef, 0, 3);
            }
            if (rr != null && rr.length >= 6) {
                System.arraycopy(rr, 3, result.velEcef, 0, 3);
            }

            float[] qr = (float[]) solClass.getField("qr").get(rtklibSol);
            if (qr != null && qr.length >= 3) {
                for (int i = 0; i < 3; i++) result.posStd[i] = qr[i];
            }
            if (qr != null && qr.length >= 6) {
                for (int i = 0; i < 3; i++) result.velStd[i] = qr[i + 3];
            }

            byte stat = solClass.getField("stat").getByte(rtklibSol);
            result.status = GnssPositionResult.SolutionStatus.fromCode(stat);

            byte ns = solClass.getField("ns").getByte(rtklibSol);
            result.numSat = ns & 0xFF;

            float age = solClass.getField("age").getFloat(rtklibSol);
            result.age = age;

            float ratio = solClass.getField("ratio").getFloat(rtklibSol);
            result.ratio = ratio;

        } catch (Exception e) {
            return result;
        }

        return result;
    }

    public static GnssPositionResult fromRtklibRtk(Object rtklibRtk) {
        GnssPositionResult result = new GnssPositionResult();
        if (rtklibRtk == null) return result;

        try {
            Class<?> rtkClass = rtklibRtk.getClass();
            Object sol = rtkClass.getField("sol").get(rtklibRtk);
            if (sol != null) {
                result = fromRtklibSol(sol);
            }

            double[] P = (double[]) rtkClass.getField("P").get(rtklibRtk);
            int nx = rtkClass.getField("nx").getInt(rtklibRtk);
            if (P != null && nx >= 6) {
                for (int i = 0; i < 3; i++) {
                    result.posCov[i * 3 + i] = P[i * nx + i];
                    result.velCov[i * 3 + i] = P[(i + 3) * nx + (i + 3)];
                }
                result.posCov[1] = P[0 * nx + 1]; result.posCov[3] = P[1 * nx + 0];
                result.posCov[2] = P[0 * nx + 2]; result.posCov[6] = P[2 * nx + 0];
                result.posCov[5] = P[1 * nx + 2]; result.posCov[7] = P[2 * nx + 1];
            }

        } catch (Exception e) {
            return result;
        }

        return result;
    }
}