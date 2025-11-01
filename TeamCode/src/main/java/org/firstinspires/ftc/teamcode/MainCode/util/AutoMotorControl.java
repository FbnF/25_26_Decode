package org.firstinspires.ftc.teamcode.MainCode.util;



import static java.lang.Thread.sleep;

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


    public static class ShooterAndFeederAction implements Action {
        private final DcMotorEx shooter;
        private final DcMotorEx intake;
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
                DcMotorEx intake,
                Servo feeder,
                double shooterPower,
                double[] feedStartS,
                double feedHoldS,
                double endPaddingS,
                double loadPos,
                double feedPos
        ) {
            this.shooter = shooter;
            this.intake = intake;
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
                intake.setPower(-0.5);
                for (int i = 0; i < 100; i++){
                    int count = i;
                }
                intake.setPower(1.0);

                initialized = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;

            // Inside any FEED window?
            boolean feeding = false;
            for (double s : feedStartS) {
                if (t >= s && t < s + feedHoldS) { feeding = true; break; }
            }
            feeder.setPosition(feeding ? feedPos  : loadPos);

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