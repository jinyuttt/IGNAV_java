package org.gnss.ignav.data;

public class Imu {

    public Imud[] data;

    public int n;

    public int nmax;

    public int decfmt;

    public int format;

    public int coor;

    public int valfmt;

    public Imu() {
        this.n = 0;
        this.nmax = 0;
        this.data = null;
        this.decfmt = 0;
        this.format = 0;
        this.coor = 0;
        this.valfmt = 0;
    }

    public void alloc(int nmax) {
        this.nmax = nmax;
        this.data = new Imud[nmax];
        for (int i = 0; i < nmax; i++) {
            data[i] = new Imud();
        }
        this.n = 0;
    }

    public void free() {
        this.data = null;
        this.n = 0;
        this.nmax = 0;
    }
}