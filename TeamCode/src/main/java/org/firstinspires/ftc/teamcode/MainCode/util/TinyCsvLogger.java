package org.firstinspires.ftc.teamcode.MainCode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/** Ultra-minimal CSV logger for TeleOp metrics. */
public final class TinyCsvLogger {
    private final BufferedWriter bw;
    private final VoltageSensor vSensor;
    private final long t0Ns;
    private final String urlHint;

    private TinyCsvLogger(BufferedWriter bw, VoltageSensor vSensor, String urlHint) {
        this.bw = bw;
        this.vSensor = vSensor;
        this.t0Ns = System.nanoTime();
        this.urlHint = urlHint;
    }

    /** Create a logger writing to FIRST/data/teleop_<stamp>.csv and emit a header. */
    public static TinyCsvLogger create(HardwareMap hw, String runTag) {
        try {
            File dir = AppUtil.ROBOT_DATA_DIR; // FIRST/data
            if (!dir.exists()) dir.mkdirs();
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String fname = "teleop_" + stamp + ".csv";
            File out = new File(dir, fname);
            BufferedWriter bw = new BufferedWriter(new FileWriter(out));
            // header
            bw.write("t_ms,tag,batt_V,launch_cmd,launch_power,launch_tps,intake_cmd,feed_pos\n");
            bw.flush();

            // pick any available voltage sensor
            VoltageSensor vs = null;
            for (VoltageSensor v : hw.getAll(VoltageSensor.class)) { vs = v; break; }

            String url = "http://192.168.43.1:8080/db/FIRST/data/" + fname;
            return new TinyCsvLogger(bw, vs, url + (runTag == null ? "" : "  tag=" + runTag));
        } catch (Exception e) {
            throw new RuntimeException("TinyCsvLogger init failed: " + e.getMessage(), e);
        }
    }

    /** Record one line. Keep calls cheap to avoid impacting loop timing. */
    public void record(String tag, double launchCmd,
                       DcMotorEx launchMotor,
                       double intakeCmd,
                       Servo feedServo) {
        try {
            long t_ms = (System.nanoTime() - t0Ns) / 1_000_000L;
            double batt = (vSensor != null) ? vSensor.getVoltage() : Double.NaN;
            double lmPower = (launchMotor != null) ? launchMotor.getPower() : Double.NaN;
            double lmTps   = (launchMotor != null) ? launchMotor.getVelocity() : Double.NaN;
            double feedPos = (feedServo != null)   ? feedServo.getPosition() : Double.NaN;

            // tag is free-form; avoid commas to keep CSV simple
            String safeTag = (tag == null) ? "" : tag.replace(",", " ");
            bw.write(String.format(Locale.US,
                    "%d,%s,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f\n",
                    t_ms, safeTag, batt, launchCmd, lmPower, lmTps, intakeCmd, feedPos));
            bw.flush(); // flush each line so partial runs still save
        } catch (Exception ignored) {
            // intentionally swallow to keep TeleOp resilient
        }
    }

    /** Close file. Safe to call multiple times. */
    public void close() {
        try { bw.close(); } catch (Exception ignored) {}
    }

    /** Convenience to show the exact download URL in telemetry once. */
    public String getUrlHint() { return urlHint; }
}