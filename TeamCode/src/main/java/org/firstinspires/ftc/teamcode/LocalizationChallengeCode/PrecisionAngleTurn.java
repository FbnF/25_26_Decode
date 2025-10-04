package org.firstinspires.ftc.teamcode.LocalizationChallengeCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Precision Angle Turn (Overshoot-Protected)", group = "Localization Challenge Code")
public class PrecisionAngleTurn extends LinearOpMode {

    // ---- Hardware ----
    DcMotor leftFront, leftRear, rightFront, rightRear;
    IMU imu;

    // ---- Tuning knobs ----
    static final double TURN_TOLERANCE_DEG   = 2.0;   // we consider “done” inside ±2°
    static final double KP_TURN              = 0.012; // proportional gain (power per degree)
    static final double MAX_TURN_POWER       = 0.35;  // clamp for approach phase
    static final double MIN_TURN_POWER       = 0.12;  // prevents stalling near target
    static final double SLOW_TURN_POWER_MAX  = 0.22;  // clamp for back-correction (after overshoot)
    static final double ERROR_DEADBAND_DEG   = 0.5;   // zero tiny errors to avoid jitter
    static final long   TIMEOUT_MS           = 7000;  // safety timeout

    // Choose your target (relative to current heading)
    static final double TARGET_RELATIVE_DEG  = 90.0;  // set to 90.0 or 180.0 etc.

    @Override
    public void runOpMode() throws InterruptedException {
        // Map motors (names must match Robot Configuration)
        leftFront  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRearMotor");

        // (Recommended) ensure +power drives the robot forward for both sides
        // rightFront.setDirection(DcMotor.Direction.REVERSE);
        // rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU init with your hub mounting (change enums if mounted differently)
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(hubOrientation));

        waitForStart();
        if (!opModeIsActive()) return;

        // Turn RELATIVE to current heading (safer than absolute)
        double startHeading  = getHeadingDeg();
        double targetHeading = normalizeAngleDeg(startHeading + TARGET_RELATIVE_DEG);

        // Controller state
        long   t0            = System.currentTimeMillis();
        double prevError     = angleErrorDeg(targetHeading, getHeadingDeg());
        boolean overshot     = false; // flips true when we cross the target (error sign change)

        while (opModeIsActive()
                && System.currentTimeMillis() - t0 < TIMEOUT_MS) {

            double heading = getHeadingDeg();
            double error   = angleErrorDeg(targetHeading, heading); // signed shortest path [-180,+180)

            // Exit when inside tolerance
            if (Math.abs(error) <= TURN_TOLERANCE_DEG) break;

            // Deadband tiny noise
            if (Math.abs(error) < ERROR_DEADBAND_DEG) error = 0;

            // Detect overshoot: sign changed and both errors nonzero
            if (!overshot && Math.signum(error) != 0 && Math.signum(prevError) != 0
                    && Math.signum(error) != Math.signum(prevError)) {
                overshot = true; // enter slow back-correction mode
            }

            // Proportional power
            double rawPower = KP_TURN * Math.abs(error);

            // Clamp depending on phase: approach vs back-correction
            double maxPower = overshot ? SLOW_TURN_POWER_MAX : MAX_TURN_POWER;

            // Ensure we never command below MIN_TURN_POWER if we need to turn
            double power = clip(rawPower, MIN_TURN_POWER, maxPower);

            // Direction: +error => need to turn CCW (left), -error => CW (right)
            double dir = Math.signum(error);

            // In-place turn: left same sign, right opposite
            setAllPower( power * dir,  power * dir, -power * dir, -power * dir);

            // Telemetry
            telemetry.addData("Start",   startHeading);
            telemetry.addData("Target",  targetHeading);
            telemetry.addData("Heading", heading);
            telemetry.addData("Error",   error);
            telemetry.addData("Phase",   overshot ? "Back-Correct (slow)" : "Approach");
            telemetry.addData("Power",   power);
            telemetry.update();

            prevError = error;
        }

        stopAll();
        sleep(150); // settle
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