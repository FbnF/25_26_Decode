package org.firstinspires.ftc.teamcode.MainCode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class ShooterConfig {
    private ShooterConfig() {} // static-only

    public static double G = 9.81;             // m/s^2
    public static double LAUNCH_DEG = 46.0;    // deg above horizontal
    public static double TARGET_H_M = 0.984;   // target center height (m)
    public static double SHOOTER_H_M = 0.248;  // shooter exit height (m)

    // Hardware
    public static double WHEEL_RADIUS_M = 0.048; // shooter wheel radius (m)
    public static double EFFICIENCY = 0.30;      // 0–1, tune on robot (slip/losses)
    public static double TICKS_PER_REV = 28.0;   // encoder ticks per revolution

    // Controls
    public static double MIN_RANGE_IN = 10.0;    // ignore ranges under this (inches)
    public static double TPS_MIN_AUTO = 800.0;   // below this, AUTO stays off
    public static double TPS_TOL = 50.0;

    public static double TPS_MAX_MECH = 2800.0;  // absolute safety limit
    public static double TPS_MAX_AUTO = 2400.0;  // clamp used by AUTO
}