package org.firstinspires.ftc.teamcode.LocalizationChallengeCode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


/*
    •   If the robot overshoots a lot, lower MAX_TURN_POWER (e.g., 0.25) or raise TURN_TOLERANCE_DEG to ±3–4°.
	•	If the yaw direction seems backwards, double-check your hub orientation enums or your motor directions.
	•	If your bot “sticks” near the target, raise MIN_TURN_POWER a little so the motors don’t stall.
	•	If you want “absolute 90°” instead of relative: set double targetHeading = 90.0; (but be aware your IMU zero may not be exactly 0).
 */

@TeleOp(name = "Turn to 90 (IMU)", group = "Localization Challenge Code")
public class TurnTo90 extends LinearOpMode {
    DcMotor leftFront, leftRear, rightFront, rightRear;
    IMU imu;

    // Tuning knobs
    static final double TURN_TOLERANCE_DEG = 2.0;   // how close to 90° is “good enough”
    static final double MIN_TURN_POWER     = 0.12;  // prevents stalling as error gets small
    static final double MAX_TURN_POWER     = 0.35;  // keeps the turn controlled
    static final double KP_TURN            = 0.01;  // simple P-gain: power per degree of error
    static final long   TIMEOUT_MS         = 6000;  // safety timeout

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRearMotor");
        // Ensure “forward” is the same direction on both sides (optional but recommended)
        // If pushing +power makes the right side go backward, uncomment these:
        // rightFront.setDirection(DcMotor.Direction.REVERSE);
        // rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        // Tell the SDK how your REV Hub is mounted so yaw/pitch/roll are correct
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(hubOrientation));

        waitForStart();
        if (!opModeIsActive()) return;

        // We will turn RELATIVE to our current heading, by +90 degrees
        double startHeading = getHeadingDeg();
        double targetHeading = normalizeAngleDeg(startHeading + 90.0);

        long t0 = System.currentTimeMillis();
        double error = angleErrorDeg(targetHeading, getHeadingDeg());

        while (opModeIsActive()
                && Math.abs(error) > TURN_TOLERANCE_DEG
                && System.currentTimeMillis() - t0 < TIMEOUT_MS) {
            // Simple proportional control: power grows with angular error, clamped
            double rawPower = KP_TURN * Math.abs(error);
            double power = clip(rawPower, MIN_TURN_POWER, MAX_TURN_POWER);
            double dir = Math.signum(error); // +error => turn left (CCW), −error => turn right (CW)
            // In-place turn: left wheels one way, right wheels the other
            setAllPower( power * dir,  power * dir, -power * dir, -power * dir);
            error = angleErrorDeg(targetHeading, getHeadingDeg());
            telemetry.addData("Start",   startHeading);
            telemetry.addData("Target",  targetHeading);
            telemetry.addData("Heading", getHeadingDeg());
            telemetry.addData("Error",   error);
            telemetry.update();
        }
        stopAll();
        sleep(150); // small settle time
    }
    // -------- Helpers --------
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
    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
