package org.gnss.ignav.adapter;

import org.gnss.ignav.constants.IgnavConstants;
import org.gnss.ignav.data.GTime;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public final class PosFileReader {

    public enum PosFormat {
        LLH,
        XYZ,
        INS_LLH,
        INS_XYZ,
        UNKNOWN
    }

    private PosFileReader() {}

    public static List<GnssPositionResult> readPosFile(String filePath) throws IOException {
        return readPosFile(filePath, null);
    }

    public static List<GnssPositionResult> readPosFile(String filePath, PosFormat formatHint) throws IOException {
        List<GnssPositionResult> results = new ArrayList<>();
        PosFormat format = formatHint;

        try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
            String line;
            while ((line = reader.readLine()) != null) {
                line = line.trim();

                if (line.isEmpty()) continue;

                if (line.startsWith("%")) {
                    if (format == null || format == PosFormat.UNKNOWN) {
                        format = detectFormatFromHeader(line);
                    }
                    continue;
                }

                GnssPositionResult result = parseLine(line, format);
                if (result != null && result.isValid()) {
                    results.add(result);
                }
            }
        }

        return results;
    }

    static PosFormat detectFormatFromHeader(String headerLine) {
        String lower = headerLine.toLowerCase();
        if (lower.contains("lat") && lower.contains("lon") && lower.contains("height")) {
            if (lower.contains("qins") || lower.contains("roll") || lower.contains("pitch") || lower.contains("yaw")) {
                return PosFormat.INS_LLH;
            }
            return PosFormat.LLH;
        }
        if (lower.contains("x-ecef") || lower.contains("y-ecef") || lower.contains("z-ecef")) {
            if (lower.contains("qins") || lower.contains("roll") || lower.contains("pitch") || lower.contains("yaw")) {
                return PosFormat.INS_XYZ;
            }
            return PosFormat.XYZ;
        }
        return PosFormat.UNKNOWN;
    }

    static GnssPositionResult parseLine(String line, PosFormat format) {
        if (format == null || format == PosFormat.UNKNOWN) {
            format = guessFormat(line);
        }
        if (format == PosFormat.UNKNOWN) return null;

        String[] parts = line.split("[,\\s]+");
        if (parts.length < 4) return null;

        GnssPositionResult result = new GnssPositionResult();

        int idx = 0;

        GTime time = parseTime(parts, idx);
        if (time == null) return null;
        result.time = time;
        idx = (parts[0].contains(":") || parts[0].contains("/")) ? (parts[1].contains(":") ? 3 : 2) : 2;

        switch (format) {
            case LLH:
                idx = parseLlh(parts, idx, result);
                break;
            case XYZ:
                idx = parseXyz(parts, idx, result);
                break;
            case INS_LLH:
                idx = parseInsLlh(parts, idx, result);
                break;
            case INS_XYZ:
                idx = parseInsXyz(parts, idx, result);
                break;
            default:
                return null;
        }

        return result;
    }

    static GTime parseTime(String[] parts, int startIdx) {
        try {
            if (parts.length < 2) return null;

            if (parts[0].contains("/") && parts.length > 1 && parts[1].contains(":")) {
                String dateStr = parts[0].trim();
                String timeStr = parts[1].trim();
                return parseDateTime(dateStr, timeStr);
            }

            if (parts[0].contains(":")) {
                String timeStr = parts[0].trim();
                if (parts.length > 1 && isNumeric(parts[1])) {
                    return parseWeekTow(parts[0], parts[1]);
                }
                return parseTimeOnly(timeStr);
            }

            if (isNumeric(parts[0]) && parts.length > 1 && isNumeric(parts[1])) {
                double tow = Double.parseDouble(parts[1]);
                int week = Integer.parseInt(parts[0]);
                return gpst2Time(week, tow);
            }

            return null;
        } catch (Exception e) {
            return null;
        }
    }

    static int parseLlh(String[] parts, int idx, GnssPositionResult result) {
        try {
            if (idx + 3 > parts.length) return idx;

            double lat = Double.parseDouble(parts[idx++]) * IgnavConstants.D2R;
            double lon = Double.parseDouble(parts[idx++]) * IgnavConstants.D2R;
            double hgt = Double.parseDouble(parts[idx++]);

            double[] pos = {lat, lon, hgt};
            double[] rr = new double[3];
            org.gnss.ignav.common.InsMath.pos2ecef(pos, rr);
            System.arraycopy(rr, 0, result.posEcef, 0, 3);

            if (idx < parts.length) {
                int stat = Integer.parseInt(parts[idx++]);
                result.status = GnssPositionResult.SolutionStatus.fromCode(stat);
            }
            if (idx < parts.length) {
                idx++;
            }
            if (idx < parts.length) {
                result.numSat = Integer.parseInt(parts[idx++]);
            }

            double[] Q = new double[9];
            if (idx + 3 <= parts.length) {
                double sdn = Double.parseDouble(parts[idx++]);
                double sde = Double.parseDouble(parts[idx++]);
                double sdu = Double.parseDouble(parts[idx++]);
                Q[4] = sdn * sdn;
                Q[0] = sde * sde;
                Q[8] = sdu * sdu;
                if (idx + 3 <= parts.length) {
                    double sdne = Double.parseDouble(parts[idx++]);
                    double sdeu = Double.parseDouble(parts[idx++]);
                    double sdun = Double.parseDouble(parts[idx++]);
                    Q[1] = Q[3] = sdne * sdne;
                    Q[2] = Q[6] = sdeu * sdeu;
                    Q[5] = Q[7] = sdun * sdun;
                }
            }
            double[] P = new double[9];
            org.gnss.ignav.common.InsMath.covecef(pos, Q, P);
            for (int i = 0; i < 3; i++) result.posStd[i] = Math.sqrt(P[i * 3 + i]);

            if (idx < parts.length) result.age = Double.parseDouble(parts[idx++]);
            if (idx < parts.length) result.ratio = Double.parseDouble(parts[idx++]);

            if (idx + 3 <= parts.length) {
                double vn = Double.parseDouble(parts[idx++]);
                double ve = Double.parseDouble(parts[idx++]);
                double vu = Double.parseDouble(parts[idx++]);
                double[] vel = new double[3];
                org.gnss.ignav.common.InsMath.enu2ecef(pos, new double[]{vn, ve, vu}, vel);
                System.arraycopy(vel, 0, result.velEcef, 0, 3);
            }

        } catch (Exception e) {
            // partial parse
        }
        return idx;
    }

    static int parseXyz(String[] parts, int idx, GnssPositionResult result) {
        try {
            if (idx + 3 > parts.length) return idx;

            result.posEcef[0] = Double.parseDouble(parts[idx++]);
            result.posEcef[1] = Double.parseDouble(parts[idx++]);
            result.posEcef[2] = Double.parseDouble(parts[idx++]);

            if (idx < parts.length) {
                int stat = Integer.parseInt(parts[idx++]);
                result.status = GnssPositionResult.SolutionStatus.fromCode(stat);
            }
            if (idx < parts.length) idx++;
            if (idx < parts.length) {
                result.numSat = Integer.parseInt(parts[idx++]);
            }

            if (idx + 3 <= parts.length) {
                result.posStd[0] = Double.parseDouble(parts[idx++]);
                result.posStd[1] = Double.parseDouble(parts[idx++]);
                result.posStd[2] = Double.parseDouble(parts[idx++]);
                if (idx + 3 <= parts.length) {
                    idx += 3;
                }
            }

            if (idx < parts.length) result.age = Double.parseDouble(parts[idx++]);
            if (idx < parts.length) result.ratio = Double.parseDouble(parts[idx++]);

            if (idx + 3 <= parts.length) {
                result.velEcef[0] = Double.parseDouble(parts[idx++]);
                result.velEcef[1] = Double.parseDouble(parts[idx++]);
                result.velEcef[2] = Double.parseDouble(parts[idx++]);
            }

        } catch (Exception e) {
            // partial parse
        }
        return idx;
    }

    static int parseInsLlh(String[] parts, int idx, GnssPositionResult result) {
        idx = parseLlh(parts, idx, result);
        return idx;
    }

    static int parseInsXyz(String[] parts, int idx, GnssPositionResult result) {
        idx = parseXyz(parts, idx, result);
        return idx;
    }

    static PosFormat guessFormat(String line) {
        String[] parts = line.split("[,\\s]+");
        int dataStart = (parts[0].contains(":") || parts[0].contains("/")) ? 2 : 2;

        if (parts.length > dataStart + 1) {
            try {
                double val1 = Double.parseDouble(parts[dataStart]);
                double val2 = Double.parseDouble(parts[dataStart + 1]);

                if (Math.abs(val1) <= 90.0 && Math.abs(val2) <= 360.0) {
                    return PosFormat.LLH;
                }
                if (Math.abs(val1) > 100.0 && Math.abs(val2) > 100.0) {
                    return PosFormat.XYZ;
                }
            } catch (NumberFormatException e) {
                return PosFormat.UNKNOWN;
            }
        }
        return PosFormat.UNKNOWN;
    }

    static boolean isNumeric(String s) {
        try {
            Double.parseDouble(s);
            return true;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    static GTime parseDateTime(String dateStr, String timeStr) {
        try {
            String[] dp = dateStr.split("[/\\-]");
            String[] tp = timeStr.split(":");
            if (dp.length < 3 || tp.length < 3) return null;

            int year = Integer.parseInt(dp[0].trim());
            int month = Integer.parseInt(dp[1].trim());
            int day = Integer.parseInt(dp[2].trim());
            int hour = Integer.parseInt(tp[0].trim());
            int minute = Integer.parseInt(tp[1].trim());
            double sec = Double.parseDouble(tp[2].trim());

            double[] ep = {year, month, day, hour, minute, sec};
            return epoch2Gps(ep);
        } catch (Exception e) {
            return null;
        }
    }

    static GTime parseTimeOnly(String timeStr) {
        return null;
    }

    static GTime parseWeekTow(String weekStr, String towStr) {
        try {
            int week = Integer.parseInt(weekStr.trim());
            double tow = Double.parseDouble(towStr.trim());
            return gpst2Time(week, tow);
        } catch (Exception e) {
            return null;
        }
    }

    static GTime gpst2Time(int week, double tow) {
        long t = IgnavConstants.GPST0_TIME + week * 604800L + (long) tow;
        double sec = tow - (long) tow;
        return new GTime(t, sec);
    }

    static GTime epoch2Gps(double[] ep) {
        int[] daysOfMonth = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        int year = (int) ep[0];
        int month = (int) ep[1];
        int day = (int) ep[2];
        int hour = (int) ep[3];
        int minute = (int) ep[4];
        double sec = ep[5];

        int doy = day;
        for (int i = 0; i < month - 1; i++) {
            doy += daysOfMonth[i];
        }
        if (month > 2 && (year % 4 == 0 && year % 100 != 0 || year % 400 == 0)) {
            doy += 1;
        }

        int gpstYearStart = 1980;
        int gpsWeek = 0;
        int dayCount = 0;

        for (int y = gpstYearStart; y < year; y++) {
            dayCount += (y % 4 == 0 && y % 100 != 0 || y % 400 == 0) ? 366 : 365;
        }
        dayCount += doy - 1;
        gpsWeek = dayCount / 7;

        int dayOfWeek = dayCount % 7;
        double tow = dayOfWeek * 86400.0 + hour * 3600.0 + minute * 60.0 + sec;

        return gpst2Time(gpsWeek, tow);
    }
}