package org.firstinspires.ftc.teamcode.LocalizationChallengeCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Encoder-IMU Hybrid Path", group = "Localization Challenge Code")
public class EncoderImuHybridPath extends LinearOpMode {

    // ---- Hardware ----
    DcMotor leftFront, leftRear, rightFront, rightRear;
    IMU imu;

    // ---- Robot constants (tune for your chassis) ----
    static final int    TICKS_PER_REV   = 537;     // e.g., goBILDA ~537.7
    static final double WHEEL_DIAM_IN   = 4.0;     // inches
    static final double GEAR_RATIO      = 1.0;     // motor->wheel (1.0 if direct)
    static final double TICKS_PER_IN    = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAM_IN);

    // ---- Motion targets/tuning ----
    static final double LEG_DISTANCE_IN     = 36.0; // distance per straight leg (inches)
    static final double BASE_POWER          = 0.30; // forward power
    static final double KP_HEADING          = 0.02; // power per degree of yaw error
    static final double MAX_CORRECTION      = 0.20; // clamp steering correction
    static final double ERROR_DEADBAND_DEG  = 0.5;  // suppress tiny jitter
    static final long   STRAIGHT_TIMEOUT_MS = 7000; // safety timeouts
    static final long   TURN_TIMEOUT_MS     = 6000;
    static final double TURN_TOLERANCE_DEG  = 2.0;
    static final double KP_TURN             = 0.012;
    static final double MAX_TURN_POWER      = 0.30;
    static final double MIN_TURN_POWER      = 0.12;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map hardware
        leftFront  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRearMotor");

        // Ensure +power drives forward on both sides (uncomment if your right side is inverted)
        // rightFront.setDirection(DcMotor.Direction.REVERSE);
        // rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Crisp stops
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Track encoders while we power the motors ourselves
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU init with hub mounting (change enums if your hub is mounted differently)
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(hubOrientation));

        waitForStart();
        if (!opModeIsActive()) return;

        // ---- Start pose and headings ----
        // Pose in inches, heading in degrees
        double x = 0.0;
        double y = 0.0;

        double startHeadingDeg = getHeadingDeg();                  // initial yaw
        double holdHeadingLeg1 = startHeadingDeg;                  // heading to hold for first straight
        double targetTurnDeg   = normalizeAngleDeg(startHeadingDeg + 90.0); // +90° relative turn

        // ---- Leg 1: drive straight with heading hold ----
        double leg1Inches = driveStraightHold(LEG_DISTANCE_IN, BASE_POWER,
                holdHeadingLeg1, STRAIGHT_TIMEOUT_MS);

        // Pose update after leg 1 (use actual distance traveled + actual heading we held)
        x += leg1Inches * Math.cos(Math.toRadians(holdHeadingLeg1));
        y += leg1Inches * Math.sin(Math.toRadians(holdHeadingLeg1));

        telemetry.addData("Leg1 inches", leg1Inches);
        telemetry.addData("Pose after Leg1", "(%.1f, %.1f)", x, y);
        telemetry.update();

        // ---- Turn to +90° relative, using IMU with proportional control ----
        turnToHeadingDeg(targetTurnDeg, TURN_TOLERANCE_DEG, TURN_TIMEOUT_MS);

        // Read actual post-turn heading (could differ slightly from target)
        double actualHeadingAfterTurn = getHeadingDeg();

        // ---- Leg 2: drive straight with heading hold at the new heading ----
        double leg2Inches = driveStraightHold(LEG_DISTANCE_IN, BASE_POWER,
                actualHeadingAfterTurn, STRAIGHT_TIMEOUT_MS);

        // Pose update after leg 2 (use actual distance + actual heading)
        x += leg2Inches * Math.cos(Math.toRadians(actualHeadingAfterTurn));
        y += leg2Inches * Math.sin(Math.toRadians(actualHeadingAfterTurn));

        // ---- Final report ----
        telemetry.addLine("----- Final Pose Estimate -----");
        telemetry.addData("Start Heading (deg)", startHeadingDeg);
        telemetry.addData("Post-Turn Heading (deg)", actualHeadingAfterTurn);
        telemetry.addData("Leg1 (in)", leg1Inches);
        telemetry.addData("Leg2 (in)", leg2Inches);
        telemetry.addData("Final (x, y) in", "(%.1f, %.1f)", x, y);
        telemetry.update();

        stopAll();
        sleep(200);
    }

    // ---- Drive straight for 'inches' while holding 'holdHeadingDeg' with IMU correction.
    // Returns the actual distance traveled (inches) computed from average encoder ticks.
    private double driveStraightHold(double inches, double basePower, double holdHeadingDeg, long timeoutMs) {
        int startLF = leftFront.getCurrentPosition();
        int startLR = leftRear.getCurrentPosition();
        int startRF = rightFront.getCurrentPosition();
        int startRR = rightRear.getCurrentPosition();

        int targetTicks = (int) Math.round(inches * TICKS_PER_IN);
        int goalLF = startLF + targetTicks;
        int goalLR = startLR + targetTicks;
        int goalRF = startRF + targetTicks;
        int goalRR = startRR + targetTicks;

        long t0 = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - t0 < timeoutMs
                && leftFront.getCurrentPosition()  < goalLF
                && leftRear.getCurrentPosition()   < goalLR
                && rightFront.getCurrentPosition() < goalRF
                && rightRear.getCurrentPosition()  < goalRR) {

            double heading = getHeadingDeg();
            double error   = angleErrorDeg(holdHeadingDeg, heading);   // signed degrees to our hold heading
            if (Math.abs(error) < ERROR_DEADBAND_DEG) error = 0.0;     // kill tiny jitter

            double correction = clip(KP_HEADING * error, -MAX_CORRECTION, MAX_CORRECTION);
            double leftPower  = clip(basePower - correction, -1.0, 1.0);  // slow left / speed right on +error
            double rightPower = clip(basePower + correction, -1.0, 1.0);

            leftFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightRear.setPower(rightPower);

            telemetry.addData("HoldHeading", holdHeadingDeg);
            telemetry.addData("Heading", heading);
            telemetry.addData("Err", error);
            telemetry.addData("Corr", correction);
            telemetry.addData("LF/LR", "%d / %d", leftFront.getCurrentPosition(), leftRear.getCurrentPosition());
            telemetry.addData("RF/RR", "%d / %d", rightFront.getCurrentPosition(), rightRear.getCurrentPosition());
            telemetry.update();
        }

        stopAll();

        // Compute actual distance from average of wheel deltas
        int dLF = leftFront.getCurrentPosition()  - startLF;
        int dLR = leftRear.getCurrentPosition()   - startLR;
        int dRF = rightFront.getCurrentPosition() - startRF;
        int dRR = rightRear.getCurrentPosition()  - startRR;

        double avgTicks = (dLF + dLR + dRF + dRR) / 4.0;
        double inchesTraveled = avgTicks / TICKS_PER_IN;

        sleep(150); // settle
        return inchesTraveled;
    }

    // ---- Turn to a target heading (deg) using IMU, proportional control, with tolerance/timeout
    private void turnToHeadingDeg(double targetHeadingDeg, double toleranceDeg, long timeoutMs) {
        long t0 = System.currentTimeMillis();
        double error = angleErrorDeg(targetHeadingDeg, getHeadingDeg());

        while (opModeIsActive()
                && Math.abs(error) > toleranceDeg
                && System.currentTimeMillis() - t0 < timeoutMs) {

            double raw = KP_TURN * Math.abs(error);
            double power = clip(raw, MIN_TURN_POWER, MAX_TURN_POWER);
            double dir = Math.signum(error);  // +error => CCW (left), -error => CW (right)

            setAllPower( power * dir,  power * dir, -power * dir, -power * dir);

            error = angleErrorDeg(targetHeadingDeg, getHeadingDeg());

            telemetry.addData("Target",  targetHeadingDeg);
            telemetry.addData("Heading", getHeadingDeg());
            telemetry.addData("Error",   error);
            telemetry.addData("Power",   power);
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

    private double normalizeAngleDeg(double angle) {
        double a = angle % 360.0;
        if (a >= 180.0) a -= 360.0;
        if (a <  -180.0) a += 360.0;
        return a;
    }

    private double angleErrorDeg(double targetDeg, double currentDeg) {
        return normalizeAngleDeg(targetDeg - currentDeg);
    }

    private void setAllPower(double lf, double lr, double rf, double rr) {
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
    }

    private void stopAll() {
        setAllPower(0, 0, 0, 0);
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}