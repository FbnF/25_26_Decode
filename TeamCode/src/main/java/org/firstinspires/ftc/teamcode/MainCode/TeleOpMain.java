package org.firstinspires.ftc.teamcode.MainCode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive; // <-- adjust path if needed
import org.firstinspires.ftc.teamcode.MainCode.util.Calculations;
import org.firstinspires.ftc.teamcode.MainCode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.MainCode.config.TagConfig;
import org.firstinspires.ftc.teamcode.MainCode.vision.AprilTagService;

@TeleOp(name = "TeleOpMain", group = "DecodeTeleopAllInOne")
public class TeleOpMain extends LinearOpMode {

    // --- Hardware ---
    private Servo feedServo;
    private MecanumDrive drive;
    private DcMotorEx intakeMotor;
    private DcMotorEx launchMotor;

    // --- Vision ---
    private AprilTagService tagService;

    // --- Drive/settings ---
    private double speedFactor = 0.5;

    // --- Intake/servo state ---
    private double intakePower = 0.0;
    private boolean isIntakeRunning = false;
    private boolean isFeedServoDown = false;

    // --- Button edge detection ---
    private boolean prevRB = false, prevDpadRight = false, prevDpadLeft = false, prevA = false;

    @Override
    public void runOpMode() {

        // Map hardware
        feedServo   = hardwareMap.get(Servo.class,    "feedServo");
        intakeMotor = hardwareMap.get(DcMotorEx.class,"IntakeMotor");
        launchMotor = hardwareMap.get(DcMotorEx.class,"LaunchMotor");

        feedServo.setPosition(0.0);
        isFeedServoDown = false;

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Drive (verify your constructor signature)
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Vision
        tagService = new AprilTagService();
        tagService.start(hardwareMap);

        waitForStart();

        // Safe startup
        intakeMotor.setPower(0.0);
        launchMotor.setVelocity(0.0);

        while (opModeIsActive()) {
            // --- Drive ---
            double axial   = -gamepad1.right_stick_y * speedFactor; // fwd/back
            double lateral =  gamepad1.left_stick_x  * speedFactor; // strafe
            double heading =  gamepad1.right_stick_x * speedFactor; // turn
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(heading, lateral), axial));

            // --- AprilTag reading (smoothed inches) ---
            AprilTagService.Reading reading = tagService.getLatest();

            if (!reading.hasTag) {
                telemetry.addLine("🟥 AprilTag: NOT DETECTED");
            } else {
                telemetry.addLine("🟩 AprilTag: DETECTED");
                telemetry.addData("Tag ID", reading.id);
                telemetry.addData("Position (in)", String.format("X: %.1f  Y: %.1f  Z: %.1f",
                        reading.xIn, reading.yIn, reading.zIn));
                telemetry.addData("Range (in)", String.format("%.1f", reading.rangeIn));
                telemetry.addData("Bearing (deg)", String.format("%.1f", reading.bearingDeg));
                telemetry.addData("Elevation (deg)", String.format("%.1f", reading.elevDeg));
                telemetry.addData("Smoothed Dist (in)", String.format("%.1f", reading.smoothedDistanceIn));
            }

            double rangeIn = reading.smoothedDistanceIn; // may be NaN if we haven’t seen a tag yet

            // --- Shooter control using config + calculations ---
            if (!Double.isNaN(rangeIn) && rangeIn > ShooterConfig.MIN_RANGE_IN) {
                double tps = Calculations.computeTPSFromRangeInches(
                        ShooterConfig.G,
                        rangeIn,
                        ShooterConfig.LAUNCH_DEG,
                        ShooterConfig.SHOOTER_H_M,
                        ShooterConfig.TARGET_H_M,
                        ShooterConfig.WHEEL_RADIUS_M,
                        ShooterConfig.EFFICIENCY,
                        ShooterConfig.TICKS_PER_REV
                );

                if (Double.isNaN(tps)) {
                    launchMotor.setVelocity(0.0);
                    telemetry.addLine("Shooter TPS invalid → motor stopped");
                } else {
                    launchMotor.setVelocity(tps);
                    telemetry.addData("TPS Target", tps);
                    telemetry.addData("TPS Measured", launchMotor.getVelocity());
                }
            } else {
                launchMotor.setVelocity(0.0);
            }

            // --- Intake toggle (RB edge) ---
            boolean rbEdge = gamepad1.right_bumper && !prevRB;
            if (rbEdge) {
                isIntakeRunning = !isIntakeRunning;
                intakeMotor.setPower(isIntakeRunning ? 0.5 : 0.0);
            }

            // --- Intake power trim with dpad (edges) ---
            boolean dpadRightEdge = gamepad1.dpad_right && !prevDpadRight;
            boolean dpadLeftEdge  = gamepad1.dpad_left  && !prevDpadLeft;
            if (dpadRightEdge)      intakePower = Math.min(1.0,  intakePower + 0.1);
            else if (dpadLeftEdge)  intakePower = Math.max(-1.0, intakePower - 0.1);
            if (isIntakeRunning)    intakeMotor.setPower(intakePower);

            // --- Feed servo toggle (A edge) ---
            boolean aEdge = gamepad1.a && !prevA;
            if (aEdge) {
                isFeedServoDown = !isFeedServoDown;
                feedServo.setPosition(isFeedServoDown ? 0.75 : 0.0);
            }

            telemetry.update();

            // Edge bookkeeping
            prevRB = gamepad1.right_bumper;
            prevDpadRight = gamepad1.dpad_right;
            prevDpadLeft = gamepad1.dpad_left;
            prevA = gamepad1.a;
        }
        // ---- cleanup runs after STOP is pressed ----
        try {
            if (launchMotor != null) launchMotor.setVelocity(0.0);
            if (intakeMotor != null) intakeMotor.setPower(0.0);
        } finally {
            // Make sure the camera is freed so the next OpMode can open it
            if (tagService != null) tagService.stop();
        }
    }
}