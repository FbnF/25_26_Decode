// ShooterConfig.java
// Full file with Option A (distance -> [TxMin, TxMax]) included.
// No REQUIRE_ALIGNED_TO_FEED flag.

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

    // --- No-shot zone (robot physically can't make it) ---
    public static double NO_SHOT_UNDER_IN = 0.0; // set this once you measure

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
            42.51,45.5,48.1, 56.9,58.25, 78.9, 84.5, 102.1, 123.5,128.13, 130.1, 134.5,136.5, 144.2,
            //58.82,83.71,85,86.46,98.69,131.18,
    };

    public static double[] TPS_AT_DIST = new double[] {
            1425.92,1440.12,1421.64 ,1423.18, 1417.68,1507.04, 1526.44, 1618.7,1742.12, 1697.36, 1754.1, 1780.68,1787, 1829.85,
            //1676, 1635, 1738, 1830, 1947, 1990, 2012, 2065,
            //1511,1615,1674,1683,1824,1933,
    };

    // ---------------- Distance -> Tx window table (Option A) ----------------
    // Same idea as TPS table: measured points, strictly increasing distance.
    // Fill these over time.
    public static boolean AUTO_ALIGN_ENABLED = true;

    // Controller knobs (dashboard-tunable)
    public static double ALIGN_KP = 0.02;
    public static double ALIGN_KD = 0.00045;
    public static double ALIGN_MAX_TURN = 0.7;
    public static double ALIGN_MIN_TURN = 0.08;
    public static double ALIGN_ERR_DEADBAND_DEG = 0.25;
    public static double ALIGN_MAX_STALE_MS = 100;

    // Distance -> allowed Tx window (degrees)
    public static double[] TX_DIST_IN = new double[] { 58.23, 78.02, 131.3};
    public static double[] TX_MIN_AT_DIST = new double[] { -8.36,-6.25,-6.39};
    public static double[] TX_MAX_AT_DIST = new double[] {  0.4,  0.91,  1.27 };

    /** Linear interpolation lookup for allowed Tx window. Returns [min, max]. */
    public static double[] lookupTxWindowFromDistanceIn(double distIn) {
        if (!Double.isFinite(distIn)) return new double[]{-999.0, 999.0};
        if (TX_DIST_IN == null || TX_MIN_AT_DIST == null || TX_MAX_AT_DIST == null) return new double[]{-999.0, 999.0};
        int n = TX_DIST_IN.length;
        if (n < 2 || TX_MIN_AT_DIST.length != n || TX_MAX_AT_DIST.length != n) return new double[]{-999.0, 999.0};

        if (distIn <= TX_DIST_IN[0]) return new double[]{TX_MIN_AT_DIST[0], TX_MAX_AT_DIST[0]};
        if (distIn >= TX_DIST_IN[n - 1]) return new double[]{TX_MIN_AT_DIST[n - 1], TX_MAX_AT_DIST[n - 1]};

        int i = 0;
        while (i < n - 1 && distIn > TX_DIST_IN[i + 1]) i++;

        double x0 = TX_DIST_IN[i];
        double x1 = TX_DIST_IN[i + 1];
        if (x1 <= x0) return new double[]{TX_MIN_AT_DIST[i], TX_MAX_AT_DIST[i]};

        double t = (distIn - x0) / (x1 - x0);

        double min = TX_MIN_AT_DIST[i] + t * (TX_MIN_AT_DIST[i + 1] - TX_MIN_AT_DIST[i]);
        double max = TX_MAX_AT_DIST[i] + t * (TX_MAX_AT_DIST[i + 1] - TX_MAX_AT_DIST[i]);
        return new double[]{min, max};
    }

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