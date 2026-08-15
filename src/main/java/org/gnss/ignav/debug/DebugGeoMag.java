package org.gnss.ignav.debug;

import org.gnss.ignav.common.GeoMag;
import org.gnss.ignav.data.BField;
import org.gnss.ignav.data.BFieldModel;

public class DebugGeoMag {
    public static void main(String[] args) {
        BFieldModel model = new BFieldModel();
        String wmmFile = "src/main/resources/geomag/WMM2015.COF";
        System.out.println("Reading WMM model from: " + wmmFile);
        int ret = GeoMag.readModel(model, wmmFile);
        System.out.println("ret=" + ret + ", nmodel=" + model.nmodel);
        System.out.println("gh1[0][1]=" + model.gh1[0][1] + ", gh1[0][2]=" + model.gh1[0][2]);
        System.out.println("gh2[0][1]=" + model.gh2[0][1] + ", gh2[0][2]=" + model.gh2[0][2]);

        BField bfield = new BField();
        double sdate = GeoMag.julday(1, 1, 2020);
        System.out.println("\nsdate=" + sdate);
        int fret = GeoMag.getFieldComponents(bfield, model, 0.0, GeoMag.UNITS_METERS,
                GeoMag.COORDSYS_GEODETIC, 30.0, 114.0, sdate);
        System.out.println("getFieldComponents ret=" + fret);
        System.out.println("d=" + bfield.d + ", i=" + bfield.i + ", h=" + bfield.h + ", f=" + bfield.f);
        System.out.println("x=" + bfield.x + ", y=" + bfield.y + ", z=" + bfield.z);

        BFieldModel model2 = new BFieldModel();
        String igrfFile = "src/main/resources/geomag/IGRF12.COF";
        System.out.println("\nReading IGRF model from: " + igrfFile);
        int ret2 = GeoMag.readModel(model2, igrfFile);
        System.out.println("ret=" + ret2 + ", nmodel=" + model2.nmodel);

        BField bfield2 = new BField();
        int fret2 = GeoMag.getFieldComponents(bfield2, model2, 0.0, GeoMag.UNITS_METERS,
                GeoMag.COORDSYS_GEODETIC, 30.0, 114.0, sdate);
        System.out.println("getFieldComponents ret=" + fret2);
        System.out.println("d=" + bfield2.d + ", i=" + bfield2.i + ", h=" + bfield2.h + ", f=" + bfield2.f);
    }
}