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

    public static boolean USE_FEED_TABLE = true;

    public static double FEED_DEFAULT = -0.9;
    public static double SIDE_DEFAULT = 1.0;


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

    public static double[] DIST_IN = new double[] {
            //54.7, 58.23, 59.7, 78.02, 78.03, 86.52, 90.51, 114.22,129.81,131.3,136.24,152.44
            //42.51,45.5,48.1, 56.9,58.25, 78.9, 84.5, 102.1, 123.5,128.13, 130.1, 134.5,136.5, 144.2,
            //58.82,83.71,85,86.46,98.69,131.18,
    };

    public static double[] TPS_AT_DIST = new double[] {
            1203 ,1331 ,1359 ,1329 ,1255 ,1292 ,1400 ,1583 ,1620 ,1670
           // 1366 ,1429 ,1367 ,1488 ,1455 ,1538 ,1559 ,1681 ,1733 ,1736 ,1794 ,1888
            //1425.92,1440.12,1421.64 ,1423.18, 1417.68,1507.04, 1526.44, 1618.7,1742.12, 1697.36, 1754.1, 1780.68,1787, 1829.85,
            //1676, 1635, 1738, 1830, 1947, 1990, 2012, 2065,
            //1511,1615,1674,1683,1824,1933,
    };

    // Distance -> allowed Tx window (degrees)
    public static double[] DIST_IN_BLUE = new double[] {
            45.2,46.12,54.4,56.2,60.2,69.7,80.5,127.4,133.47,139.24
            // 54.7, 58.23, 59.7, 78.02, 78.03, 86.52, 90.51, 114.22,129.81,131.3,136.24,152.44
    };
    //added 3 to all the mins
    public static double[] MIN_ANGLE_BLUE = new double[] {
            0.4688888889 ,0.4688888889 ,0.4688888889 ,0.4688888889 ,0.4688888889 ,0.4688888889 ,0.4688888889 ,0.4688888889 ,0.4688888889 ,0.4688888889
            //-11.1 ,-8.36 ,-9.81 ,-6.25,-8.02,-10.65 ,-8.7 ,-8.8 ,-5.4 ,-6.39 ,-0.21 ,-5.72
             };
    public static double[] MAX_ANGLE_BLUE = new double[] {
            0.4688888889 ,0.4688888889 ,0.4688888889 ,0.4688888889,0.4688888889,0.4688888889 ,0.4688888889 ,0.4688888889 ,0.4688888889 ,0.4688888889
            //0.801,0.4,3.45,0.91,4.64 ,-1.45 ,-2.69 ,-2.6,1.14,1.27,0.08,-1.68
             };
    public static double[] FEED_POWER_AT_DIST_BLUE = new double[] {
            -0.9 ,-0.9 ,-0.4 ,-0.49 ,-0.6 ,-0.4 ,-0.2 ,-0.17 ,-0.15 ,-0.15
    };
    public static double[] SIDE_POWER_AT_DIST_BLUE = new double[] {
            1 ,1 ,0.7 ,1 ,0.5,0.5,0.5
    };


    //Red Side maps
    public static double[] TX_DIST_IN_RED = new double[] {
            45.82, 48.78,51.15,58.22,62.8,69.09,81.5, 130.9 ,136 ,138.6 // 54.7,58.23,59.7,78.02,78.03,86.52,90.51,114.22,129.81,131.3,136.24,152.44

    };
    //added 3 to all the mins
    public static double[] TX_MIN_AT_DIST_RED = new double[] {
            -5.06 ,-10 ,-4.35 ,-6.9 ,-4.42 ,-11.33 ,-7.33,-7.6 ,-5.2 ,-5.4 //-14.5, -13 ,-12 ,-10.5 ,-10.25 ,-9.5 ,-9 ,-8.5 ,-6.5 ,-6.25 ,-5 ,-4

    };
    public static double[] TX_MAX_AT_DIST_RED = new double[] {
            3.16 ,3 ,2.55 ,2 ,0.01 ,-1.3 ,-4.08 ,-3.8 ,-3.61 ,-3.4// 1, 1.25,1.5,3.5,3.25 ,2, -1, -2 ,1.5,1.25 ,1 ,-1.5
    };
    public static double[] FEED_POWER_AT_DIST_RED = new double[] {
            -0.9, -0.4 ,-0.25 ,-0.49 ,-0.25 ,-0.4 ,-0.2,-0.15,-0.15,-0.15
    };
    public static double[] SIDE_POWER_AT_DIST_RED = new double[] {
            1 ,1 ,0.7 ,1 ,0.5,0.5,0.5,0.5,0.5,0.5
    };
    public static double[] DIST_IN_RED = new double[] {
            45.82, 48.78,51.15,58.22,62.8,69.09,81.5, 130.9 ,136 ,138.6// -14.5, -13 ,-12 ,-10.5 ,-10.25 ,-9.5 ,-9 ,-8.5 ,-6.5 ,-6.25 ,-5 ,-4
            };

    public static double[] TPS_AT_DIST_RED = new double[] {
            1203 ,1331 ,1359 ,1260 ,1255 ,1292 ,1361 ,1583 ,1610 ,1670
            };


    public static double getFeedPowerAtDistanceRED(double distIn) {
        if (!Double.isFinite(distIn)) return 0.0;
        if (DIST_IN_RED == null || FEED_POWER_AT_DIST_RED == null) return 0.0;
        if (DIST_IN_RED.length < 2 || FEED_POWER_AT_DIST_RED.length != DIST_IN_RED.length) return 0.0;

        if (distIn <= DIST_IN_RED[0]) return FEED_POWER_AT_DIST_RED[0];
        int last = DIST_IN_RED.length - 1;
        if (distIn >= DIST_IN_RED[last]) return FEED_POWER_AT_DIST_RED[last];

        int i = 0;
        while (i < last && distIn > DIST_IN_RED[i + 1]) i++;

        double x0 = DIST_IN_RED[i];
        double x1 = DIST_IN_RED[i + 1];
        double y0 = FEED_POWER_AT_DIST_RED[i];
        double y1 = FEED_POWER_AT_DIST_RED[i + 1];

        if (x1 <= x0) return y0;

        double t = (distIn - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }
    public static double getSidePowerAtDistanceRED(double distIn) {
        if (!Double.isFinite(distIn)) return 0.0;
        if (DIST_IN_RED == null || SIDE_POWER_AT_DIST_RED == null) return 0.0;
        if (DIST_IN_RED.length < 2 || SIDE_POWER_AT_DIST_RED.length != DIST_IN_RED.length) return 0.0;

        if (distIn <= DIST_IN_RED[0]) return SIDE_POWER_AT_DIST_RED[0];
        int last = DIST_IN_RED.length - 1;
        if (distIn >= DIST_IN_RED[last]) return SIDE_POWER_AT_DIST_RED[last];

        int i = 0;
        while (i < last && distIn > DIST_IN_RED[i + 1]) i++;

        double x0 = DIST_IN_RED[i];
        double x1 = DIST_IN_RED[i + 1];
        double y0 = SIDE_POWER_AT_DIST_RED[i];
        double y1 = SIDE_POWER_AT_DIST_RED[i + 1];

        if (x1 <= x0) return y0;

        double t = (distIn - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    public static double getFeedPowerAtDistanceBLUE(double distIn) {
        if (!Double.isFinite(distIn)) return 0.0;
        if (DIST_IN_BLUE == null || FEED_POWER_AT_DIST_BLUE == null) return 0.0;
        if (DIST_IN_BLUE.length < 2 || FEED_POWER_AT_DIST_BLUE.length != DIST_IN_BLUE.length) return 0.0;

        if (distIn <= DIST_IN_BLUE[0]) return FEED_POWER_AT_DIST_BLUE[0];
        int last = DIST_IN_BLUE.length - 1;
        if (distIn >= DIST_IN_BLUE[last]) return FEED_POWER_AT_DIST_BLUE[last];

        int i = 0;
        while (i < last && distIn > DIST_IN_BLUE[i + 1]) i++;

        double x0 = DIST_IN_BLUE[i];
        double x1 = DIST_IN_BLUE[i + 1];
        double y0 = FEED_POWER_AT_DIST_BLUE[i];
        double y1 = FEED_POWER_AT_DIST_BLUE[i + 1];

        if (x1 <= x0) return y0;

        double t = (distIn - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }
    public static double getSidePowerAtDistanceBLUE(double distIn) {
        if (!Double.isFinite(distIn)) return 0.0;
        if (DIST_IN_BLUE == null || SIDE_POWER_AT_DIST_BLUE == null) return 0.0;
        if (DIST_IN_BLUE.length < 2 || SIDE_POWER_AT_DIST_BLUE.length != DIST_IN_BLUE.length) return 0.0;

        if (distIn <= DIST_IN_BLUE[0]) return SIDE_POWER_AT_DIST_BLUE[0];
        int last = DIST_IN_BLUE.length - 1;
        if (distIn >= DIST_IN_BLUE[last]) return SIDE_POWER_AT_DIST_BLUE[last];

        int i = 0;
        while (i < last && distIn > DIST_IN_BLUE[i + 1]) i++;

        double x0 = DIST_IN_BLUE[i];
        double x1 = DIST_IN_BLUE[i + 1];
        double y0 = SIDE_POWER_AT_DIST_BLUE[i];
        double y1 = SIDE_POWER_AT_DIST_BLUE[i + 1];

        if (x1 <= x0) return y0;

        double t = (distIn - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    public static double[] lookupTxWindowFromDistanceInRED(double distIn) {
        if (!Double.isFinite(distIn)) return new double[]{-999.0, 999.0};
        if (TX_DIST_IN_RED == null || TX_MIN_AT_DIST_RED == null || TX_MAX_AT_DIST_RED == null) return new double[]{-999.0, 999.0};
        int n = TX_DIST_IN_RED.length;
        if (n < 2 || TX_MIN_AT_DIST_RED.length != n || TX_MAX_AT_DIST_RED.length != n) return new double[]{-999.0, 999.0};

        if (distIn <= TX_DIST_IN_RED[0]) return new double[]{TX_MIN_AT_DIST_RED[0], TX_MAX_AT_DIST_RED[0]};
        if (distIn >= TX_DIST_IN_RED[n - 1]) return new double[]{TX_MIN_AT_DIST_RED[n - 1], TX_MAX_AT_DIST_RED[n - 1]};

        int i = 0;
        while (i < n - 1 && distIn > TX_DIST_IN_RED[i + 1]) i++;

        double x0 = TX_DIST_IN_RED[i];
        double x1 = TX_DIST_IN_RED[i + 1];
        if (x1 <= x0) return new double[]{TX_MIN_AT_DIST_RED[i], TX_MAX_AT_DIST_RED[i]};

        double t = (distIn - x0) / (x1 - x0);

        double min = TX_MIN_AT_DIST_RED[i] + t * (TX_MIN_AT_DIST_RED[i + 1] - TX_MIN_AT_DIST_RED[i]);
        double max = TX_MAX_AT_DIST_RED[i] + t * (TX_MAX_AT_DIST_RED[i + 1] - TX_MAX_AT_DIST_RED[i]);
        return new double[]{min, max};
    }

    /** Linear interpolation lookup. */
    public static double lookupTpsFromDistanceInRED(double distIn) {
        if (!Double.isFinite(distIn)) return 0.0;
        if (DIST_IN_RED == null || TPS_AT_DIST_RED == null) return 0.0;
        if (DIST_IN_RED.length < 2 || TPS_AT_DIST_RED.length != DIST_IN_RED.length) return 0.0;

        if (distIn <= DIST_IN_RED[0]) return TPS_AT_DIST_RED[0];
        int last = DIST_IN_RED.length - 1;
        if (distIn >= DIST_IN_RED[last]) return TPS_AT_DIST_RED[last];

        int i = 0;
        while (i < last && distIn > DIST_IN_RED[i + 1]) i++;
        double x0 = DIST_IN_RED[i];
        double x1 = DIST_IN_RED[i + 1];
        double y0 = TPS_AT_DIST_RED[i];
        double y1 = TPS_AT_DIST_RED[i + 1];

        if (x1 <= x0) return y0;

        double t = (distIn - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    /** Linear interpolation lookup for allowed Tx window. Returns [min, max]. */
    public static double[] lookupTxWindowFromDistanceIn(double distIn) {
        if (!Double.isFinite(distIn)) return new double[]{-999.0, 999.0};
        if (DIST_IN_BLUE == null || MIN_ANGLE_BLUE == null || MAX_ANGLE_BLUE == null) return new double[]{-999.0, 999.0};
        int n = DIST_IN_BLUE.length;
        if (n < 2 || MIN_ANGLE_BLUE.length != n || MAX_ANGLE_BLUE.length != n) return new double[]{-999.0, 999.0};

        if (distIn <= DIST_IN_BLUE[0]) return new double[]{MIN_ANGLE_BLUE[0], MAX_ANGLE_BLUE[0]};
        if (distIn >= DIST_IN_BLUE[n - 1]) return new double[]{MIN_ANGLE_BLUE[n - 1], MAX_ANGLE_BLUE[n - 1]};

        int i = 0;
        while (i < n - 1 && distIn > DIST_IN_BLUE[i + 1]) i++;

        double x0 = DIST_IN_BLUE[i];
        double x1 = DIST_IN_BLUE[i + 1];
        if (x1 <= x0) return new double[]{MIN_ANGLE_BLUE[i], MAX_ANGLE_BLUE[i]};

        double t = (distIn - x0) / (x1 - x0);

        double min = MIN_ANGLE_BLUE[i] + t * (MIN_ANGLE_BLUE[i + 1] - MIN_ANGLE_BLUE[i]);
        double max = MAX_ANGLE_BLUE[i] + t * (MAX_ANGLE_BLUE[i + 1] - MAX_ANGLE_BLUE[i]);
        return new double[]{min, max};
    }

    /** Linear interpolation lookup. */
    public static double lookupTpsFromDistanceIn(double distIn) {
        if (!Double.isFinite(distIn)) return 0.0;
        if (DIST_IN_BLUE == null || TPS_AT_DIST == null) return 0.0;
        if (DIST_IN_BLUE.length < 2 || TPS_AT_DIST.length != DIST_IN_BLUE.length) return 0.0;

        if (distIn <= DIST_IN_BLUE[0]) return TPS_AT_DIST[0];
        int last = DIST_IN_BLUE.length - 1;
        if (distIn >= DIST_IN_BLUE[last]) return TPS_AT_DIST[last];

        int i = 0;
        while (i < last && distIn > DIST_IN_BLUE[i + 1]) i++;

        double x0 = DIST_IN_BLUE[i];
        double x1 = DIST_IN_BLUE[i + 1];
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