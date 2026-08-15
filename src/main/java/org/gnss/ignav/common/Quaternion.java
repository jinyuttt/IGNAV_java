package org.gnss.ignav.common;

public final class Quaternion {

    public double w;
    public double x;
    public double y;
    public double z;

    public Quaternion() {
        this.w = 1.0;
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
    }

    public Quaternion(double w, double x, double y, double z) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static final Quaternion IDENTITY = new Quaternion(1.0, 0.0, 0.0, 0.0);

    private static final double ZERO_TOLERANCE = 1e-6;

    public static Quaternion copy(Quaternion qi) {
        return new Quaternion(qi.w, qi.x, qi.y, qi.z);
    }

    public void copyFrom(Quaternion qi) {
        this.w = qi.w;
        this.x = qi.x;
        this.y = qi.y;
        this.z = qi.z;
    }

    public void set(double w, double x, double y, double z) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double len() {
        return Math.sqrt(w * w + x * x + y * y + z * z);
    }

    public static double len(Quaternion q) {
        return Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    }

    public void normalizeSelf() {
        double n = len();
        if (n > 0.0) {
            w /= n;
            x /= n;
            y /= n;
            z /= n;
        }
    }

    public static Quaternion normalize(Quaternion qi) {
        Quaternion qo = copy(qi);
        qo.normalizeSelf();
        return qo;
    }

    public static void normalize(Quaternion qo, Quaternion qi) {
        double n = len(qi);
        if (n > 0.0) {
            qo.w = qi.w / n;
            qo.x = qi.x / n;
            qo.y = qi.y / n;
            qo.z = qi.z / n;
        } else {
            qo.copyFrom(qi);
        }
    }

    public static Quaternion conj(Quaternion qi) {
        return new Quaternion(qi.w, -qi.x, -qi.y, -qi.z);
    }

    public static void conj(Quaternion qo, Quaternion qi) {
        qo.w = qi.w;
        qo.x = -qi.x;
        qo.y = -qi.y;
        qo.z = -qi.z;
    }

    public static Quaternion inv(Quaternion qi) {
        return new Quaternion(qi.w, -qi.x, -qi.y, -qi.z);
    }

    public static void inv(Quaternion qi, Quaternion qo) {
        qo.w = qi.w;
        qo.x = -qi.x;
        qo.y = -qi.y;
        qo.z = -qi.z;
    }

    public static Quaternion mul(Quaternion q1, Quaternion q2) {
        Quaternion o = new Quaternion();
        o.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
        o.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
        o.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
        o.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
        return o;
    }

    public static void mul(Quaternion o, Quaternion q1, Quaternion q2) {
        o.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
        o.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
        o.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
        o.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    }

    public static void scale(Quaternion qo, Quaternion qi, double f) {
        qo.w = qi.w * f;
        qo.x = qi.x * f;
        qo.y = qi.y * f;
        qo.z = qi.z * f;
    }

    public static void add(Quaternion qo, Quaternion q1, Quaternion q2) {
        qo.w = q1.w + q2.w;
        qo.x = q1.x + q2.x;
        qo.y = q1.y + q2.y;
        qo.z = q1.z + q2.z;
    }

    public static double dot(Quaternion q1, Quaternion q2) {
        return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    }

    public static void init(Quaternion q, double ax, double ay, double az, double mx, double my, double mz) {
        double initRoll = Math.atan2(-ay, -az);
        double initPitch = Math.atan2(ax, -az);
        double cosRoll = Math.cos(initRoll);
        double sinRoll = Math.sin(initRoll);
        double cosPitch = Math.cos(initPitch);
        double sinPitch = Math.sin(initPitch);
        double magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
        double magY = my * cosRoll - mz * sinRoll;
        double initYaw = Math.atan2(-magY, magX);

        cosRoll = Math.cos(initRoll * 0.5);
        sinRoll = Math.sin(initRoll * 0.5);
        cosPitch = Math.cos(initPitch * 0.5);
        sinPitch = Math.sin(initPitch * 0.5);
        double cosHeading = Math.cos(initYaw * 0.5);
        double sinHeading = Math.sin(initYaw * 0.5);

        q.w = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
        q.x = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
        q.y = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
        q.z = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
    }

    public static void initAxis(Quaternion q, double x, double y, double z, double a) {
        double a2 = a * 0.5;
        double s = Math.sin(a2);
        q.x = x * s;
        q.y = y * s;
        q.z = z * s;
        q.w = Math.cos(a2);
    }

    public static void initAxisV(Quaternion q, double[] v, double a) {
        initAxis(q, v[0], v[1], v[2], a);
    }

    public static void toAxis(Quaternion q, double[] xyz, double[] angle) {
        double ang = 2.0 * Math.acos(q.w);
        double s = Math.sqrt(1.0 - q.w * q.w);
        if (s < ZERO_TOLERANCE) {
            angle[0] = 0.0;
            xyz[0] = 1.0;
            xyz[1] = 0.0;
            xyz[2] = 0.0;
        } else {
            angle[0] = ang;
            xyz[0] = ang * q.x / s;
            xyz[1] = ang * q.y / s;
            xyz[2] = ang * q.z / s;
        }
    }

    public static void rotVec(double[] vo, double[] vi, Quaternion q) {
        double vx = vi[0], vy = vi[1], vz = vi[2];
        double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
        double qww = qw * qw, qxx = qx * qx, qyy = qy * qy, qzz = qz * qz;
        double qwx = qw * qx, qwy = qw * qy, qwz = qw * qz;
        double qxy = qx * qy, qxz = qx * qz, qyz = qy * qz;
        vo[0] = (qww + qxx - qyy - qzz) * vx + 2.0 * ((qxy - qwz) * vy + (qxz + qwy) * vz);
        vo[1] = (qww - qxx + qyy - qzz) * vy + 2.0 * ((qxy + qwz) * vx + (qyz - qwx) * vz);
        vo[2] = (qww - qxx - qyy + qzz) * vz + 2.0 * ((qxz - qwy) * vx + (qyz + qwx) * vy);
    }

    public static void rotVecSelf(double[] v, Quaternion q) {
        double[] vo = new double[3];
        rotVec(vo, v, q);
        System.arraycopy(vo, 0, v, 0, 3);
    }

    public static void toEuler(double[] euler, Quaternion q) {
        double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z, ww = q.w * q.w;
        euler[2] = normalizeEuler02Pi(Math.atan2(2.0 * (q.x * q.y + q.z * q.w), xx - yy - zz + ww));
        euler[1] = Math.asin(-2.0 * (q.x * q.z - q.y * q.w));
        euler[0] = Math.atan2(2.0 * (q.y * q.z + q.x * q.w), -xx - yy + zz + ww);
    }

    public static void toRhRotMatrix(Quaternion q, double[] m) {
        InsMath.quatToRhRotMatrix(new double[]{q.w, q.x, q.y, q.z}, m);
    }

    public static void fromDcm(double[] C, Quaternion q) {
        double[] qa = new double[4];
        InsMath.dcm2quat(C, qa);
        q.w = qa[0];
        q.x = qa[1];
        q.y = qa[2];
        q.z = qa[3];
    }

    public static void toDcm(Quaternion q, double[] C) {
        InsMath.quat2dcm(new double[]{q.w, q.x, q.y, q.z}, C);
    }

    public static void lerp(Quaternion qo, Quaternion qfrom, Quaternion qto, double t) {
        double scale0 = 1.0 - t;
        double scale1 = t;
        qo.x = scale0 * qfrom.x + scale1 * qto.x;
        qo.y = scale0 * qfrom.y + scale1 * qto.y;
        qo.z = scale0 * qfrom.z + scale1 * qto.z;
        qo.w = scale0 * qfrom.w + scale1 * qto.w;
    }

    public static void nlerp(Quaternion qo, Quaternion qfrom, Quaternion qto, double t) {
        lerp(qo, qfrom, qto, t);
        qo.normalizeSelf();
    }

    public static void slerp(Quaternion qo, Quaternion qfrom, Quaternion qto, double t) {
        double cosom = dot(qfrom, qto);
        if (cosom >= 1.0) {
            qo.copyFrom(qfrom);
            return;
        }
        Quaternion to1 = new Quaternion();
        if (cosom < 0.0) {
            cosom = -cosom;
            to1.x = -qto.x;
            to1.y = -qto.y;
            to1.z = -qto.z;
            to1.w = -qto.w;
        } else {
            to1.copyFrom(qto);
        }
        double scale0, scale1;
        if (cosom < 0.99995) {
            double omega = Math.acos(cosom);
            double sinom = Math.sin(omega);
            scale0 = Math.sin((1.0 - t) * omega) / sinom;
            scale1 = Math.sin(t * omega) / sinom;
        } else {
            scale0 = 1.0 - t;
            scale1 = t;
        }
        qo.x = scale0 * qfrom.x + scale1 * to1.x;
        qo.y = scale0 * qfrom.y + scale1 * to1.y;
        qo.z = scale0 * qfrom.z + scale1 * to1.z;
        qo.w = scale0 * qfrom.w + scale1 * to1.w;
    }

    public static void eulerToQuat(double[] euler, Quaternion q) {
        double cosRoll = Math.cos(euler[0] * 0.5);
        double sinRoll = Math.sin(euler[0] * 0.5);
        double cosPitch = Math.cos(euler[1] * 0.5);
        double sinPitch = Math.sin(euler[1] * 0.5);
        double cosHeading = Math.cos(euler[2] * 0.5);
        double sinHeading = Math.sin(euler[2] * 0.5);
        Quaternion tmp = new Quaternion();
        tmp.w = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
        tmp.x = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
        tmp.y = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
        tmp.z = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
        normalize(q, tmp);
    }

    public static void rpy2quat(double[] rpy, Quaternion q) {
        eulerToQuat(rpy, q);
    }

    public static void quat2rpy(Quaternion q, double[] rpy) {
        toEuler(rpy, q);
    }

    public void applyRelativeYawPitchRoll(double yaw, double pitch, double roll) {
        Quaternion qyaw = new Quaternion(), qpitch = new Quaternion(), qroll = new Quaternion();
        Quaternion qrot = new Quaternion(), q1 = new Quaternion(), q2 = new Quaternion();
        Quaternion q3 = new Quaternion(), q4 = new Quaternion();

        initAxis(qyaw, 0.0, 0.0, 1.0, yaw);
        initAxis(qpitch, 0.0, 1.0, 0.0, pitch);
        initAxis(qroll, 1.0, 0.0, 0.0, roll);

        mul(q1, qyaw, qpitch);
        mul(qrot, q1, qroll);

        mul(q1, this, qrot);
        conj(q2, this);
        mul(q3, q1, q2);
        mul(q4, q3, this);
        q4.normalizeSelf();
        this.copyFrom(q4);
    }

    public static void fromU2V(Quaternion q, double[] u, double[] v, double[] up) {
        double[] n = new double[3];
        InsMath.cross3(u, v, n);
        double d = InsMath.dot(u, v, 3);
        if (InsMath.norm(n, 3) < ZERO_TOLERANCE) {
            if (d < 0.0) {
                double[] axis = new double[3];
                if (up != null) {
                    System.arraycopy(up, 0, axis, 0, 3);
                } else {
                    axis[0] = 1.0;
                    axis[1] = 0.0;
                    axis[2] = 0.0;
                }
                initAxis(q, axis[0], axis[1], axis[2], Math.PI);
            } else {
                q.set(1.0, 0.0, 0.0, 0.0);
            }
            return;
        }
        double angle = Math.atan2(InsMath.norm(n, 3), d);
        double nn = InsMath.norm(n, 3);
        n[0] /= nn;
        n[1] /= nn;
        n[2] /= nn;
        initAxis(q, n[0], n[1], n[2], angle);
    }

    private static double normalizeEuler02Pi(double a) {
        a = a % (2.0 * Math.PI);
        if (a < 0.0) a += 2.0 * Math.PI;
        return a;
    }

    public double[] toArray() {
        return new double[]{w, x, y, z};
    }

    public static Quaternion fromArray(double[] a) {
        return new Quaternion(a[0], a[1], a[2], a[3]);
    }

    @Override
    public String toString() {
        return String.format("Quaternion[w=%.6f, x=%.6f, y=%.6f, z=%.6f]", w, x, y, z);
    }
}