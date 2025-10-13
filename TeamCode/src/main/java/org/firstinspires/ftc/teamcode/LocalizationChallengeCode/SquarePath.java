package org.firstinspires.ftc.teamcode.LocalizationChallengeCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Square Path (Optimized)", group = "Localization Challenge Code")
public class SquarePath extends LinearOpMode {

    // Drive hardware
    DcMotor leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;
    IMU imu;

    // Robot-specific constants (tune for your bot)
    static final double WHEEL_DIAMETER_IN     = 4.0;  // inches
    static final double GEAR_RATIO            = 1.0;
    static final int    TICKS_PER_REV         = 537;  // goBILDA 5202 series ~537.7
    static final double DISTANCE_PER_SIDE_IN  = 24.0; // inches per side

    static final double DRIVE_SPEED           = 0.35; // Power for driving
    static final double TURN_SPEED            = 0.25; // (kept for readability; P-control clamps below)
    static final double TURN_TOLERANCE_DEG    = 2.0;  // +/- degrees for turn accuracy

    static final double TICKS_PER_IN = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    // P-Control tuning for turning
    static final double KP_TURN               = 0.01;
    static final double MIN_TURN_POWER        = 0.12;
    static final double MAX_TURN_POWER        = 0.35;
    static final long   TURN_TIMEOUT_MS       = 4000; // Safety timeout for turn

    @Override
    public void runOpMode() throws InterruptedException {
        // --- 1. Hardware Mapping ---
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftRearMotor   = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightRearMotor  = hardwareMap.get(DcMotor.class, "rightRearMotor");

        // --- 2. Motor Setup ---
        // Ensure "forward" is consistent (adjust setDirection as needed)
        // rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        // rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- 3. IMU Setup ---
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(hubOrientation));

        // Always reset encoders right before start to ensure relative moves are accurate
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Start using encoders for continuous position tracking
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialization Complete. Ready for Square.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // --- 4. Square Path Execution ---
        // Track an absolute target heading, adding 90° each side.
        double currentTargetHeading = getHeadingDeg();

        for (int side = 0; side < 4 && opModeIsActive(); side++) {

            // A. DRIVE FORWARD
            telemetry.addData("Path", "Driving Side %d...", side + 1);
            telemetry.update();
            driveForwardInches(DISTANCE_PER_SIDE_IN, DRIVE_SPEED);

            // B. CALCULATE NEW TURN TARGET
            currentTargetHeading = normalizeAngleDeg(currentTargetHeading + 90.0);

            // C. TURN 90 DEGREES
            telemetry.addData("Path", "Turning to %.1f...", currentTargetHeading);
            telemetry.update();
            turnToHeadingDeg(currentTargetHeading, TURN_TOLERANCE_DEG);

            telemetry.addData("Side Complete", side + 1);
            telemetry.update();
            sleep(500); // Small pause between moves
        }

        // 5. Final Stop
        stopAll();
        telemetry.addData("Path", "Square Complete!");
        telemetry.update();
        sleep(2000);
    }

    // ---- Helper Methods (Updated Drive) ----

    /** Drives the robot forward the specified distance using RUN_TO_POSITION mode. */
    private void driveForwardInches(double inches, double power) {
        int ticks = (int) Math.round(inches * TICKS_PER_IN);

        // Build per-motor targets RELATIVE to each current position
        int lfTarget = leftFrontMotor.getCurrentPosition()  + ticks;
        int lrTarget = leftRearMotor.getCurrentPosition()   + ticks;
        int rfTarget = rightFrontMotor.getCurrentPosition() + ticks;
        int rrTarget = rightRearMotor.getCurrentPosition()  + ticks;

        // Apply targets
        leftFrontMotor.setTargetPosition(lfTarget);
        leftRearMotor.setTargetPosition(lrTarget);
        rightFrontMotor.setTargetPosition(rfTarget);
        rightRearMotor.setTargetPosition(rrTarget);

        // Switch to RUN_TO_POSITION mode
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start driving
        setMotorPower(power);

        // Wait until ALL motors reach their targets (or STOP pressed)
        while (opModeIsActive() && allBusy()) {
            telemetry.addData("Status", "Driving");
            telemetry.addData("LF cur/target", "%d / %d", leftFrontMotor.getCurrentPosition(),  lfTarget);
            telemetry.addData("LR cur/target", "%d / %d", leftRearMotor.getCurrentPosition(),   lrTarget);
            telemetry.addData("RF cur/target", "%d / %d", rightFrontMotor.getCurrentPosition(), rfTarget);
            telemetry.addData("RR cur/target", "%d / %d", rightRearMotor.getCurrentPosition(),  rrTarget);
            telemetry.update();
        }

        // Stop and revert to RUN_USING_ENCODER
        stopAll();
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(150);
    }

    /** Turns the robot to an absolute target heading using simple P-control. */
    private void turnToHeadingDeg(double targetHeadingDeg, double toleranceDeg) {
        long t0 = System.currentTimeMillis();
        double error = angleErrorDeg(targetHeadingDeg, getHeadingDeg());

        while (opModeIsActive()
                && Math.abs(error) > toleranceDeg
                && System.currentTimeMillis() - t0 < TURN_TIMEOUT_MS) {

            double rawPower = KP_TURN * Math.abs(error);
            double power = clip(rawPower, MIN_TURN_POWER, MAX_TURN_POWER);
            double dir = Math.signum(error); // +error => CCW (left), −error => CW (right)

            // In-place turn: left wheels one way, right wheels the other
            setAllPower( power * dir,  power * dir, -power * dir, -power * dir);

            error = angleErrorDeg(targetHeadingDeg, getHeadingDeg());

            telemetry.addData("Target",  targetHeadingDeg);
            telemetry.addData("Heading", getHeadingDeg());
            telemetry.addData("Error",   error);
            telemetry.update();
        }

        stopAll();
        sleep(150);
    }

    // --- Utility Methods ---

    private void setMotorMode(DcMotor.RunMode mode) {
        leftFrontMotor.setMode(mode);
        leftRearMotor.setMode(mode);
        rightFrontMotor.setMode(mode);
        rightRearMotor.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontMotor.setZeroPowerBehavior(behavior);
        leftRearMotor.setZeroPowerBehavior(behavior);
        rightFrontMotor.setZeroPowerBehavior(behavior);
        rightRearMotor.setZeroPowerBehavior(behavior);
    }

    private void setMotorPower(double power) {
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);
    }

    /** Returns true only if ALL motors are still busy. */
    private boolean allBusy() {
        return leftFrontMotor.isBusy()
                && leftRearMotor.isBusy()
                && rightFrontMotor.isBusy()
                && rightRearMotor.isBusy();
    }

    private void setAllPower(double lf, double lr, double rf, double rr) {
        leftFrontMotor.setPower(lf);
        leftRearMotor.setPower(lr);
        rightFrontMotor.setPower(rf);
        rightRearMotor.setPower(rr);
    }

    private void stopAll() {
        setAllPower(0, 0, 0, 0);
    }

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

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}