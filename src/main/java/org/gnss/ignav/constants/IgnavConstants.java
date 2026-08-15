package org.gnss.ignav.constants;

public final class IgnavConstants {

    private IgnavConstants() {}

    public static final double CLIGHT = 299792458.0;
    public static final double OMGE = 7.2921151467E-5;
    public static final double RE_WGS84 = 6378137.0;
    public static final double FE_WGS84 = 1.0 / 298.257223563;
    public static final long GPST0_TIME = 315964800L;
    public static final double HION = 350000.0;

    public static final int MAXFREQ = 7;

    public static final double FREQ1 = 1.57542E9;
    public static final double FREQ2 = 1.22760E9;
    public static final double FREQ5 = 1.17645E9;
    public static final double FREQ6 = 1.27875E9;
    public static final double FREQ7 = 1.20714E9;
    public static final double FREQ8 = 1.191795E9;
    public static final double FREQ9 = 2.492028E9;
    public static final double FREQ1_GLO = 1.60200E9;
    public static final double DFRQ1_GLO = 0.56250E6;
    public static final double FREQ2_GLO = 1.24600E9;
    public static final double DFRQ2_GLO = 0.43750E6;
    public static final double FREQ3_GLO = 1.202025E9;
    public static final double FREQ1_CMP = 1.561098E9;
    public static final double FREQ2_CMP = 1.20714E9;
    public static final double FREQ3_CMP = 1.26852E9;

    public static final double EFACT_GPS = 1.0;
    public static final double EFACT_GLO = 1.5;
    public static final double EFACT_GAL = 1.0;
    public static final double EFACT_QZS = 1.0;
    public static final double EFACT_CMP = 1.0;
    public static final double EFACT_IRN = 1.5;
    public static final double EFACT_SBS = 3.0;

    public static final int SYS_NONE = 0x00;
    public static final int SYS_GPS = 0x01;
    public static final int SYS_SBS = 0x02;
    public static final int SYS_GLO = 0x04;
    public static final int SYS_GAL = 0x08;
    public static final int SYS_QZS = 0x10;
    public static final int SYS_CMP = 0x20;
    public static final int SYS_IRN = 0x40;
    public static final int SYS_LEO = 0x80;
    public static final int SYS_ALL = 0xFF;

    public static final int TSYS_GPS = 0;
    public static final int TSYS_UTC = 1;
    public static final int TSYS_GLO = 2;
    public static final int TSYS_GAL = 3;
    public static final int TSYS_QZS = 4;
    public static final int TSYS_CMP = 5;
    public static final int TSYS_IRN = 6;

    public static final int NFREQ = 3;
    public static final int NFREQGLO = 2;
    public static final int NEXOBS = 10;

    public static final int MINPRNGPS = 1;
    public static final int MAXPRNGPS = 32;
    public static final int NSATGPS = MAXPRNGPS - MINPRNGPS + 1;

    public static final int MINPRNGLO = 1;
    public static final int MAXPRNGLO = 27;
    public static final int NSATGLO = MAXPRNGLO - MINPRNGLO + 1;

    public static final int MINPRNGAL = 1;
    public static final int MAXPRNGAL = 30;
    public static final int NSATGAL = MAXPRNGAL - MINPRNGAL + 1;

    public static final int MINPRNQZS = 193;
    public static final int MAXPRNQZS = 199;
    public static final int MINPRNQZS_S = 183;
    public static final int MAXPRNQZS_S = 189;
    public static final int NSATQZS = MAXPRNQZS - MINPRNQZS + 1;

    public static final int MINPRNCMP = 1;
    public static final int MAXPRNCMP = 46;
    public static final int NSATCMP = MAXPRNCMP - MINPRNCMP + 1;

    public static final int MINPRNIRN = 1;
    public static final int MAXPRNIRN = 14;
    public static final int NSATIRN = MAXPRNIRN - MINPRNIRN + 1;

    public static final int MINPRNSBS = 120;
    public static final int MAXPRNSBS = 142;
    public static final int NSATSBS = MAXPRNSBS - MINPRNSBS + 1;

    public static final int NSAT = NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN + NSATSBS;
    public static final int MAXSAT = NSAT;
    public static final int MAXCODE = 30;
    public static final int MAXSTA = 200;
    public static final int MAXOBS = 64;
    public static final int MAXRCV = 32;

    public static final int NUMSYS = 7;

    public static final int SOLQ_NONE = -1;
    public static final int SOLQ_FIX = 0;
    public static final int SOLQ_FLOAT = 1;
    public static final int SOLQ_SPP = 2;
    public static final int SOLQ_DGPS = 3;
    public static final int SOLQ_SBAS = 4;
    public static final int SOLQ_PPP = 5;
    public static final int SOLQ_INS = 6;
    public static final int SOLQ_INS_GNSS = 7;
    public static final int SOLQ_INS_GNSS_TC = 8;
    public static final int SOLQ_DR = 9;
    public static final int SOLQ_AR = 10;
    public static final int SOLQ_VO = 11;
    public static final int SOLQ_GRTH = 12;
    public static final int SOLQ_WAAS = 13;
    public static final int SOLQ_PROP = 14;
    public static final int SOLQ_OMIN = 15;
    public static final int SOLQ_MONO = 16;
    public static final int SOLQ_STEREO = 17;
    public static final int MAXSOLQ = 17;

    public static final int IONOOPT_OFF = 0;
    public static final int IONOOPT_BRDC = 1;
    public static final int IONOOPT_SBAS = 2;
    public static final int IONOOPT_IFLC = 3;
    public static final int IONOOPT_EST = 4;
    public static final int IONOOPT_TEC = 5;
    public static final int IONOOPT_QZS = 6;
    public static final int IONOOPT_LEX = 7;
    public static final int IONOOPT_STEC = 8;

    public static final int TROPOPT_OFF = 0;
    public static final int TROPOPT_SAAS = 1;
    public static final int TROPOPT_SBAS = 2;
    public static final int TROPOPT_EST = 3;
    public static final int TROPOPT_ESTG = 4;
    public static final int TROPOPT_ZTD = 5;

    public static final int EPHOPT_BRDC = 0;
    public static final int EPHOPT_PREC = 1;
    public static final int EPHOPT_SBAS = 2;
    public static final int EPHOPT_SSRAPC = 3;
    public static final int EPHOPT_SSRCOM = 4;
    public static final int EPHOPT_LEX = 5;

    public static final int ARMODE_OFF = 0;
    public static final int ARMODE_CONT = 1;
    public static final int ARMODE_INST = 2;
    public static final int ARMODE_FIXHOLD = 3;
    public static final int ARMODE_PPPAR = 4;
    public static final int ARMODE_PPPAR_ILS = 5;
    public static final int ARMODE_WLNL = 6;
    public static final int ARMODE_TCAR = 7;
    public static final int ARMODE_WLNLC = 8;
    public static final int ARMODE_TCARC = 9;

    public static final int PMODE_SINGLE = 0;
    public static final int PMODE_DGPS = 1;
    public static final int PMODE_KINEMA = 2;
    public static final int PMODE_STATIC = 3;
    public static final int PMODE_MOVEB = 4;
    public static final int PMODE_FIXED = 5;
    public static final int PMODE_PPP_KINEMA = 6;
    public static final int PMODE_PPP_STATIC = 7;
    public static final int PMODE_PPP_FIXED = 8;

    public static final double PI = Math.PI;
    public static final double D2R = PI / 180.0;
    public static final double R2D = 180.0 / PI;

    public static final double MU = 3.986004418E14;
    public static final double J2 = 1.082627E-3;
    public static final double WGS_E = 0.0818191908425;
    public static final double RP = 6356752.31425;
    public static final double E_SQR = 0.00669437999014;

    public static final int INS_BAOFF = 0;
    public static final int INS_BAEST = 1;
    public static final int INS_BGOFF = 0;
    public static final int INS_BGEST = 1;
    public static final int INS_RGEST = 1;
    public static final int INS_RAEST = 1;

    public static final double MG2M = 1E-3 * 9.7803253359;
    public static final double DEG2R = D2R / 3600.0;

    public static final int INSS_NONE = 0;
    public static final int INSS_INIT = 0;
    public static final int INSS_ALIGN = 1;
    public static final int INSS_MECH = 2;
    public static final int INSS_LCUD = 3;
    public static final int INSS_TCUD = 4;
    public static final int INSS_TIME = 5;
    public static final int INSS_REBOOT = 6;
    public static final int INSS_LACK = 7;
    public static final int INSS_ZVU = 8;
    public static final int INSS_ZARU = 9;
    public static final int INSS_NHC = 10;
    public static final int INSS_ODO = 11;
    public static final int INSS_RTS = 12;
    public static final int INSS_MAGH = 13;

    public static final int NPOS = 5;

    public static final int IMUDETST_GLRT = 0;
    public static final int IMUDETST_MV = 1;
    public static final int IMUDETST_MAG = 2;
    public static final int IMUDETST_ARE = 3;
    public static final int IMUDETST_ALL = 4;

    public static final int INSUPD_INSS = 0;
    public static final int INSUPD_TIME = 1;
    public static final int INSUPD_MEAS = 2;

    public static final int INS_MECH_NED = 0;
    public static final int INS_MECH_ECEF = 1;

    public static final int INS_ALIGN_COARSE = 0;
    public static final int INS_ALIGN_FINE = 1;
    public static final int INSALIGN_CORSE = 1;
    public static final int INSALIGN_FINE = 2;
    public static final int INSALIGN_FINEEX = 3;
    public static final int INSALIGN_LARGE = 4;
    public static final int INSALIGN_DEFAULT = 5;
    public static final int INSALIGN_VELMATCH = 6;
    public static final int INSALIGN_SINGLE = 7;
    public static final int INSALIGN_PPK = 8;
    public static final int INSALIGN_DGPS = 9;
    public static final int INSALIGN_RTK = 10;
    public static final int INSALIGN_OFF = 11;
    public static final int INS_ALIGN_FINEEX = 2;
    public static final int INS_ALIGN_FINE_LYM = 3;
    public static final int INS_ALIGN_VEL_MATCH = 4;

    public static final int INS_LC = 0;
    public static final int INS_TC = 1;

    public static final int INS_UPDATE_OK = 0;
    public static final int INS_UPDATE_FAIL = -1;

    public static final int MAXIMU = 5;
    public static final int MAXIMUBUF = 36000;

    public static final int MAXERRMSG = 1024;
}