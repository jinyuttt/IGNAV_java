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

    @Override
    public String toString() {
        return "GTime{time=" + time + ", sec=" + sec + "}";
    }
}