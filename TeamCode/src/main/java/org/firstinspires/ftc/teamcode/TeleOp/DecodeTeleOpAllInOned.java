package org.firstinspires.ftc.teamcode.TeleOp;

// Ensure all necessary imports are present (from the sample code)
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.MecanumDrive; // Make sure this import is correct for your project

import java.util.List;

@Disabled
@TeleOp(group = "DecodeTeleopAllInOne")
public class DecodeTeleOpAllInOned extends LinearOpMode {
    double intakePower;
    Servo feedServo;
    MecanumDrive drive;
    double speedFactor;
    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
    private VisionPortal visionPortal;

    private AprilTagProcessor aprilTag;

    DcMotorEx intakeMotor;
    DcMotorEx launchMotor;
    double launchPower;

    //shooter velocity constants
    double g = 9.8; //m/s^2
    double x;
    double Theta = 46 * Math.PI / 180;
    double HGoal = 0.984;//in meters
    double HShoot = 0.248;//in meters

    double denominator;
    double numerator;
    double effiencyFactor = 0.3;

    double VelOfShooter;
    //  double Vtip;

    double Radius = 0.048;

    double PulsePerRev = 28;

    double RPM;

    boolean isIntakeRunning;

    boolean isFeedServoDown;

    boolean isFastMode;

    double TargetTicksPerSecond = 0;


    double Vtip;

    @Override
    public void runOpMode() {


        feedServo = hardwareMap.get(Servo.class, "feedServo");
        // Initialize the drive class


        feedServo.setPosition(0);
        isFeedServoDown = false;
        isFastMode = false;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //DcMotorEx shooterMotor;
        speedFactor = 0.5;

        initAprilTag();


        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        launchMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        isIntakeRunning = false;
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the start button to be pressed
        waitForStart();
        intakeMotor.setPower(0);
        launchMotor.setPower(0);

        // Ensure the op mode is active and the robot is not interrupted
        while (opModeIsActive()) {
            // Get input from the gamepad
            double axial = -gamepad1.right_stick_y * speedFactor; // Invert the y-axis
            double lateral = -gamepad1.left_stick_x * speedFactor;  // Strafe is x-axis
            double heading = -gamepad1.right_stick_x * speedFactor;
            PoseVelocity2d drivePower = new PoseVelocity2d(
                    new Vector2d(
                            heading,
                            lateral
                    ),
                    axial

            );


            //Set drive powers
            drive.setDrivePowers(drivePower);


            x = telemetryAprilTag();
            telemetry.addData("distance", x);
            telemetry.update();
            if (gamepad2.a) {
                if (x > 10) {
                    x = x * 0.0254;
                    numerator = g * Math.pow(x, 2);
                    denominator = 2 * Math.pow(Math.cos(Theta), 2) * (x * Math.tan(Theta) - (HShoot - HGoal));
                    VelOfShooter = Math.sqrt(numerator / denominator);
                    RPM = (60 * VelOfShooter) / (2 * Math.PI * Radius * effiencyFactor);
                    // Vtip = RPM * (2 * Math.PI * Radius) / 60;
                    TargetTicksPerSecond = RPM * (PulsePerRev / 60);
                    launchMotor.setVelocity(TargetTicksPerSecond);

                    telemetry.addData("distance", x);
                    telemetry.addData("numerator", numerator);
                    telemetry.addData("denominator", denominator);
                    telemetry.addData("VelOfShooter", VelOfShooter);
                    telemetry.addData("RPM", RPM);
                    telemetry.addData("TPS CaLc", TargetTicksPerSecond);
                    telemetry.addData("TPS Measured", launchMotor.getVelocity());
                    telemetry.update();
                } else {
                    VelOfShooter = 0;
                    launchMotor.setVelocity(0);

                }
            }

            if (gamepad1.a) {
                if (isFastMode) {
                    speedFactor = 0.5;
                } else {
                    speedFactor = 0.8;
                }
                isFastMode = !isFastMode;
            }


            //intake system
            if (gamepad2.rightBumperWasPressed()) {
                if (isIntakeRunning) {
                    intakeMotor.setPower(0);
                } else {
                    intakeMotor.setPower(0.7);
                }
                isIntakeRunning = !isIntakeRunning;
            }
            /*
            if(gamepad1.dpadRightWasPressed()){
                if (intakePower - 0.1 == -1.0){
                    intakePower = -1.0;
                }
                else{
                    intakePower=intakePower+0.1;
                }

            }
            else if (gamepad1.dpadLeftWasPressed()) {
                if (intakePower + 0.1 < 0){
                    intakePower = 0;
                }
                else {
                    intakePower=intakePower-0.1;
                }
            }
            intakeMotor.setPower(intakePower);
               */

            if (gamepad2.b) {
                if (isFeedServoDown) {
                    feedServo.setPosition(0);
                } else {
                    feedServo.setPosition(0.75);
                }
                isFeedServoDown = !isFeedServoDown;
            }

            //launch speed
            //launch system


            //   drive.updatePoseEstimate();
           /* telemetry.addData("Axial (Forward/Back)", axial);
            telemetry.addData("Lateral (Strafe)", lateral);
            telemetry.addData("Heading (Turn)", heading);
            telemetry.addData("Launch speed", launchMotor.getPower());
            telemetry.addData("is intake running?", isIntakeRunning);*/
            telemetry.update();


            // Create a pose2d from the gamepad input
            // The first parameter is the forward/backward vector, the second is the strafe vector
            // The third parameter is the rotational vector
            //drive.setWeightedDrivePower(new Pose2d(y, x, rx));

            // Update the drive's pose (needed for any position-based actions later)
            //drive.update();

            // You can add telemetry for debugging here if needed
            // Pose2d pose = drive.getPoseEstimate();
            // telemetry.addData("x", pose.getX());
            // telemetry.addData("y", pose.getY());
            // telemetry.addData("heading", Math.toDegrees(pose.getHeading()));
            // telemetry.update();
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private double telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.isEmpty()) {
            telemetry.addLine("🟥 AprilTag: NOT DETECTED");
            telemetry.addLine("Make sure the tag is visible to the camera.");
            return (-1);
        }

        telemetry.addLine("🟩 AprilTag: DETECTED");
        telemetry.addData("Total Tags Seen", detections.size());


        for (AprilTagDetection tag : detections) {
            // Map tag ID to name
            String tagName;
            switch (tag.id) {
                case 21:
                    tagName = "GPP (ID 21)";
                    break;
                case 22:
                    tagName = "PGP (ID 22)";
                    break;
                case 23:
                    tagName = "PPG (ID 23)";
                    break;
                default:
                    tagName = "Unknown (" + tag.id + ")";
                    break;
            }

            telemetry.addLine("------------------------------------");
            telemetry.addData("Tag", tagName);

            if (tag.metadata != null) {
                telemetry.addData("Position (in)",
                        String.format("X: %.1f  Y: %.1f  Z: %.1f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                //telemetry.addData("Orientation (deg)",
                //String.format("Yaw: %.1f  Pitch: %.1f  Roll: %.1f",
                // tag.ftcPose.yaw, tag.ftcPose.pitch, tag.ftcPose.roll));
                //telemetry.addData("Range/Bearing/Elev",
                //String.format("%.1f in, %.1f°, %.1f°",
                //tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
            } //else {
            //telemetry.addData("Tag Center (px)",
            //String.format("(%.0f, %.0f)", tag.center.x, tag.center.y));
            //}
            return (tag.ftcPose.y);

        }
        return (0);
    }
}

