package org.firstinspires.ftc.teamcode.MainCode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Reusable Road Runner Actions:
 *  - setMotorPower(...) : one-shot, non-blocking motor power setter
 *  - ShooterAndFeederAction : spins shooter, pulses feed servo via schedule, then stops shooter
 *
 * IMPORTANT (your RR flavor): Action.run() returns TRUE to keep running, FALSE when finished.
 */
public final class AutoMotorControl {

    private AutoMotorControl() {} // no instances

    /** One-shot action that sets a motor's power and immediately completes (non-blocking). */
    public static Action setMotorPower(DcMotor m, double power) {
        return (TelemetryPacket pkt) -> {
            if (m != null) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(power);
            }
            return false; // finished immediately; does NOT block follower
        };
    }

    /**
     * Shooter + Feeder combined action (time-window schedule).
     * - On first tick: starts shooter, parks servo at loadPos.
     * - While running: toggles between feedPos/loadPos whenever elapsed time is within any window.
     * - On finish: parks servo at loadPos, stops shooter.
     *
     * @param shooter        DcMotorEx shooter (can be null if you only want servo pulses)
     * @param feeder         Servo feeder (positional)
     * @param shooterPower   open-loop power for shooter (swap to velocity if you prefer)
     * @param feedStartS     start times (seconds from action start) for each pulse
     * @param feedHoldS      hold time at FEED for each pulse (seconds)
     * @param endPaddingS    extra LOAD time after last pulse before completing (seconds)
     * @param loadPos        servo position for "load/park"
     * @param feedPos        servo position for "feed"
     */
    public static class ShooterAndFeederAction implements Action {
        private final DcMotorEx shooter;
        private final Servo feeder;
        private final double shooterPower;
        private final double[] feedStartS;
        private final double feedHoldS;
        private final double endPaddingS;
        private final double loadPos;
        private final double feedPos;

        private boolean initialized = false;
        private long t0;

        public ShooterAndFeederAction(
                DcMotorEx shooter,
                Servo feeder,
                double shooterPower,
                double[] feedStartS,
                double feedHoldS,
                double endPaddingS,
                double loadPos,
                double feedPos
        ) {
            this.shooter = shooter;
            this.feeder = feeder;
            this.shooterPower = shooterPower;
            this.feedStartS = feedStartS;
            this.feedHoldS = feedHoldS;
            this.endPaddingS = endPaddingS;
            this.loadPos = loadPos;
            this.feedPos = feedPos;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                t0 = System.nanoTime();
                if (shooter != null) {
                    shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shooter.setPower(shooterPower);
                }
                feeder.setPosition(loadPos);
                initialized = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;

            // Inside any FEED window?
            boolean feeding = false;
            for (double s : feedStartS) {
                if (t >= s && t < s + feedHoldS) { feeding = true; break; }
            }
            feeder.setPosition(feeding ? feedPos : loadPos);

            // Telemetry (optional dashboard insight)
            packet.put("t_s", String.format("%.2f", t));
            packet.put("feeding", feeding);

            double lastEnd = (feedStartS.length > 0 ? feedStartS[feedStartS.length - 1] : 0.0)
                    + feedHoldS + endPaddingS;

            // TRUE = keep running, FALSE = done (per your RR build)
            if (t < lastEnd) return true;

            // Finish
            feeder.setPosition(loadPos);
            if (shooter != null) shooter.setPower(0.0);
            return false;
        }
    }
}