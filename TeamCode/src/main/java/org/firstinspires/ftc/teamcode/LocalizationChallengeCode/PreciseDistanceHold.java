package org.firstinspires.ftc.teamcode.LocalizationChallengeCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Precise Distance (Encoders + IMU Hold)", group = "Localization Challenge Code")
public class PreciseDistanceHold extends LinearOpMode {

    // ---- Hardware ----
    DcMotor leftFront, leftRear, rightFront, rightRear;
    IMU imu;

    // ---- Tuning knobs (adjust for your robot) ----
    static final int    TICKS_PER_REV   = 537;     // e.g., goBILDA ~537.7
    static final double WHEEL_DIAM_IN   = 4.0;     // wheel diameter (in)
    static final double GEAR_RATIO      = 1.0;     // motor->wheel ratio (1.0 if direct)
    static final double TICKS_PER_IN    = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAM_IN);

    static final double TARGET_DISTANCE_IN = 48.0; // drive distance (inches)
    static final double BASE_POWER         = 0.30; // forward power baseline
    static final double KP_HEADING         = 0.02; // power per degree of yaw error
    static final double MAX_CORRECTION     = 0.20; // clamp correction magnitude
    static final double ERROR_DEADBAND_DEG = 0.5;  // zero-out tiny errors to avoid jitter
    static final long   TIMEOUT_MS         = 7000; // safety timeout

    @Override
    public void runOpMode() throws InterruptedException {
        // Map motors (names must match Robot Configuration)
        leftFront  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRearMotor");

        // Make "forward = positive" for both sides (uncomment if your right side runs backward on +power)
        // rightFront.setDirection(DcMotor.Direction.REVERSE);
        // rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Crisp stopping at targets
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // We use RUN_USING_ENCODER because we are manually setting the power based on IMU correction.
        // RUN_TO_POSITION is not suitable here because it takes control of power away from the OpMode.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU init with correct hub mounting (change enums if your hub is mounted differently)
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(hubOrientation));

        waitForStart();
        if (!opModeIsActive()) return;

        // Build RELATIVE tick targets (safer than comparing to a fixed literal)
        int startLF = leftFront.getCurrentPosition();
        int startLR = leftRear.getCurrentPosition();
        int startRF = rightFront.getCurrentPosition();
        int startRR = rightRear.getCurrentPosition();

        int targetTicks = (int) Math.round(TARGET_DISTANCE_IN * TICKS_PER_IN);
        int goalLF = startLF + targetTicks;
        int goalLR = startLR + targetTicks;
        int goalRF = startRF + targetTicks;
        int goalRR = startRR + targetTicks;

        // Hold the starting heading while we drive straight
        double holdHeading = getHeadingDeg();
        long t0 = System.currentTimeMillis();

        // Loop while motors haven't reached the target distance
        while (opModeIsActive()
                && System.currentTimeMillis() - t0 < TIMEOUT_MS
                // Check if ALL motors have reached their goal (using the weakest link logic)
                && leftFront.getCurrentPosition()  < goalLF
                && leftRear.getCurrentPosition()   < goalLR
                && rightFront.getCurrentPosition() < goalRF
                && rightRear.getCurrentPosition()  < goalRR) {

            double heading  = getHeadingDeg();
            double errorDeg = angleErrorDeg(holdHeading, heading); // +error => we’re rotated one way relative to holdHeading

            // Deadband tiny errors to reduce twitch
            if (Math.abs(errorDeg) < ERROR_DEADBAND_DEG) errorDeg = 0.0;

            // Calculate steering correction:
            // If errorDeg is positive, we need more power on the right side and less on the left side (to turn CCW back to center).
            double correction = clip(KP_HEADING * errorDeg, -MAX_CORRECTION, MAX_CORRECTION);

            double leftPower  = clip(BASE_POWER - correction, -1.0, 1.0);
            double rightPower = clip(BASE_POWER + correction, -1.0, 1.0);

            // Apply power
            leftFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightRear.setPower(rightPower);

            // Telemetry feedback
            telemetry.addData("HoldHeading", holdHeading);
            telemetry.addData("Heading", heading);
            telemetry.addData("Error(deg)", errorDeg);
            telemetry.addData("Correction", correction);
            telemetry.addData("L/R power", "%.2f / %.2f", leftPower, rightPower);
            telemetry.addData("LF", "%d / %d", leftFront.getCurrentPosition(),  goalLF);
            telemetry.addData("LR", "%d / %d", leftRear.getCurrentPosition(),   goalLR);
            telemetry.addData("RF", "%d / %d", rightFront.getCurrentPosition(), goalRF);
            telemetry.addData("RR", "%d / %d", rightRear.getCurrentPosition(),  goalRR);
            telemetry.update();
        }

        stopAll();
        sleep(150);
    }

    // ---- Helpers ----
    private double getHeadingDeg() {
        YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
        return a.getYaw(AngleUnit.DEGREES);
    }

    private double angleErrorDeg(double targetDeg, double currentDeg) {
        double e = (targetDeg - currentDeg) % 360.0;
        if (e >= 180.0) e -= 360.0;
        if (e <  -180.0) e += 360.0;
        return e;
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void stopAll() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
