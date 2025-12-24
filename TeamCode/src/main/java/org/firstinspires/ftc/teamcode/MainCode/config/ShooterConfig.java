package org.firstinspires.ftc.teamcode.MainCode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class ShooterConfig {
    private ShooterConfig() {} // static-only

    // ---------------- Controls / Limits ----------------
    public static double MIN_RANGE_IN = 10.0;    // ignore ranges under this (inches)
    public static double TPS_MIN_AUTO = 800.0;   // below this, AUTO stays off
    public static double TPS_TOL = 50.0;

    public static double TPS_MAX_MECH = 2800.0;  // absolute safety limit
    public static double TPS_MAX_AUTO = 2400.0;  // clamp used by AUTO

    // ---------------- ONE dashboard mode switch ----------------
    // true  = MATCH MODE: use table (ignores TPS_SCALE/OFFSET)
    // false = TUNING MODE: use physics + TPS_SCALE/OFFSET
    public static boolean USE_TABLE = true;

    // ---------------- Dashboard tuning knobs (used ONLY in physics mode) ----------------
    // Final TPS = (physicsTPS * TPS_SCALE) + TPS_OFFSET
    public static double TPS_SCALE = 1.0;
    public static double TPS_OFFSET = 0.0;

    // Optional: smooth distance to reduce jitter (0 = no smoothing, 1 = heavy smoothing)
    public static double DIST_SMOOTH_ALPHA = 0.20;

    // ---------------- Physics model params ----------------
    public static double G = 9.81;             // m/s^2
    public static double LAUNCH_DEG = 46.0;    // deg above horizontal
    public static double TARGET_H_M = 0.984;   // target center height (m)
    public static double SHOOTER_H_M = 0.248;  // shooter exit height (m)

    // Hardware
    public static double WHEEL_RADIUS_M = 0.048; // shooter wheel radius (m)
    public static double EFFICIENCY = 0.30;      // start here; TPS_OFFSET handles leftover bias
    public static double TICKS_PER_REV = 28.0;

    // ---------------- Distance -> TPS table ----------------
    // Replace with measured points over time.
    // Must be same length and DIST_IN strictly increasing.
    public static double[] DIST_IN = new double[] {
            58.82,83.71,85,86.46,98.69,131.18,
    };

    public static double[] TPS_AT_DIST = new double[] {
            1511,1615,1674,1683,1824,1933,
    };

    /** Linear interpolation lookup. */
    public static double lookupTpsFromDistanceIn(double distIn) {
        if (!Double.isFinite(distIn)) return 0.0;
        if (DIST_IN == null || TPS_AT_DIST == null) return 0.0;
        if (DIST_IN.length < 2 || TPS_AT_DIST.length != DIST_IN.length) return 0.0;

        if (distIn <= DIST_IN[0]) return TPS_AT_DIST[0];
        int last = DIST_IN.length - 1;
        if (distIn >= DIST_IN[last]) return TPS_AT_DIST[last];

        int i = 0;
        while (i < last && distIn > DIST_IN[i + 1]) i++;

        double x0 = DIST_IN[i];
        double x1 = DIST_IN[i + 1];
        double y0 = TPS_AT_DIST[i];
        double y1 = TPS_AT_DIST[i + 1];

        if (x1 <= x0) return y0;

        double t = (distIn - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    /** Clamp ONLY (used for table mode). */
    public static double clampTps(double tpsIn) {
        if (!Double.isFinite(tpsIn)) return 0.0;
        double tps = tpsIn;
        tps = Math.min(tps, TPS_MAX_AUTO);
        tps = Math.min(tps, TPS_MAX_MECH);
        if (tps < TPS_MIN_AUTO) return 0.0;
        return tps;
    }

    /** Applies scale/offset and clamps (used for physics tuning mode). */
    public static double applyTuningAndClamp(double baseTps) {
        if (!Double.isFinite(baseTps)) return 0.0;
        double tps = (baseTps * TPS_SCALE) + TPS_OFFSET;
        return clampTps(tps);
    }
}