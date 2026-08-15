package org.gnss.ignav.adapter;

import org.gnss.ignav.data.GTime;

public interface GnssObservationProvider {

    boolean hasObservation();

    GTime getObservationTime();

    int processObservation();

    GnssPositionResult getPositionResult();

    double[] getObservationResiduals();

    double[] getDesignMatrix();

    int getObservationCount();

    int getSatelliteCount();
}