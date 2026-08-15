package org.gnss.ignav.data;

public class GTime {

    public long time;

    public double sec;

    public GTime() {
        this.time = 0;
        this.sec = 0.0;
    }

    public GTime(long time, double sec) {
        this.time = time;
        this.sec = sec;
    }

    public GTime(GTime other) {
        this.time = other.time;
        this.sec = other.sec;
    }

    public boolean equals(GTime other) {
        return this.time == other.time && Math.abs(this.sec - other.sec) < 1e-12;
    }

    public int compareTo(GTime other) {
        if (this.time < other.time) return -1;
        if (this.time > other.time) return 1;
        if (this.sec < other.sec) return -1;
        if (this.sec > other.sec) return 1;
        return 0;
    }

    public static GTime zero() {
        return new GTime(0, 0.0);
    }

    public static double timeDiff(GTime t1, GTime t2) {
        return (t1.time - t2.time) + (t1.sec - t2.sec);
    }

    public static void time2epoch(GTime t, double[] ep) {
        long[] daysOfMonth = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        long doy, mday, day;
        int year, month, dayOfMonth;
        double sod;

        long gpst0 = 315964800L;
        long unixTime = t.time + gpst0;
        long totalDays = unixTime / 86400;
        sod = unixTime % 86400 + t.sec;

        long daysSince0 = totalDays + 719527;

        long quad = daysSince0 / 146097;
        long rem = daysSince0 % 146097;
        long y400 = quad * 400;
        long y100 = rem / 36524;
        rem = rem % 36524;
        long y4 = rem / 1461;
        rem = rem % 1461;
        long y1 = rem / 365;
        rem = rem % 365;

        year = (int) (y400 + y100 * 100 + y4 * 4 + y1);
        if (y1 == 4 || y100 == 4) year--;

        long prevLeapDays = year / 4 - year / 100 + year / 400;
        doy = daysSince0 - (365 * year + prevLeapDays);

        boolean isLeap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
        if (isLeap) daysOfMonth[1] = 29;
        else daysOfMonth[1] = 28;

        month = 0;
        for (int i = 0; i < 12; i++) {
            if (doy < daysOfMonth[i]) {
                month = i + 1;
                break;
            }
            doy -= daysOfMonth[i];
        }
        if (month == 0) month = 12;

        dayOfMonth = (int) (doy + 1);

        ep[0] = year;
        ep[1] = month;
        ep[2] = dayOfMonth;
        ep[3] = (int) (sod / 3600);
        ep[4] = (int) ((sod % 3600) / 60);
        ep[5] = sod % 60;
    }

    @Override
    public String toString() {
        return "GTime{time=" + time + ", sec=" + sec + "}";
    }
}