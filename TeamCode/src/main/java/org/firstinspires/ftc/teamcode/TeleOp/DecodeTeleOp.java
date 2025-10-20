package org.firstinspires.ftc.teamcode.TeleOp;

// Ensure all necessary imports are present (from the sample code)
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.TeleOp.DistanceCalc;
//import org.firstinspires.ftc.teamcode.MecanumDrive; // Make sure this import is correct for your project

import java.util.List;

@TeleOp(group = "DecodeTeleop")
public class DecodeTeleOp extends LinearOpMode {
    //MecanumDrive drive;
    //double speedFactor;

    double ticksPerSec;
    double intakePower;
    double launchPower;
    DcMotorEx intakeMotor;
    DcMotorEx launchMotor;
    boolean isIntakeRunning;
    @Override
    public void runOpMode() {
        // Initialize the drive class

        //drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //speedFactor = 0.5;



         launchPower = 0.0;
         intakePower = 0.3;
         intakeMotor=hardwareMap.get(DcMotorEx.class, "IntakeMotor");
         launchMotor=hardwareMap.get(DcMotorEx.class,"LaunchMotor");
         launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         isIntakeRunning = true;
         intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the start button to be pressed
        waitForStart();
        intakeMotor.setPower(intakePower);
        launchMotor.setPower(launchPower);

        // Ensure the op mode is active and the robot is not interrupted
        while (opModeIsActive()) {
            // Get input from the gamepad
            //double axial = -gamepad1.right_stick_y * speedFactor; // Invert the y-axis
            //double lateral = -gamepad1.left_stick_x * speedFactor;  // Strafe is x-axis
            //double heading = -gamepad1.right_stick_x * speedFactor;
            //ticksPerSec = DistanceCalc.DistanceCalc();
            //telemetry.addData("tps", ticksPerSec);
            //telemetry.addData("distance calc", DistanceCalc.DistanceCalc());
            // Rotation is x-axis of right stick


            //intake system
            if(gamepad1.rightBumperWasPressed()){
                if(isIntakeRunning){
                    intakeMotor.setPower(0);
                } else {
                    intakeMotor.setPower(0.5);
                }
                isIntakeRunning = !isIntakeRunning;
            }
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

            //launch speed
            if(gamepad1.dpadUpWasPressed()){
                if (launchPower - 0.1 == -1.0){
                    launchPower = -1.0;
                }
                else{
                    launchPower=launchPower+0.1;
                }

            }
            else if (gamepad1.dpadDownWasPressed()) {
                if (launchPower + 0.1 < 0){
                    launchPower = 0;
                }
                else {
                    launchPower=launchPower-0.1;
                }
            }
            launchMotor.setPower(launchPower);


            //launch system
/*
            PoseVelocity2d drivePower = new PoseVelocity2d(
                    new Vector2d(
                            axial,
                            lateral
                    ),
                    heading

            );

 */
            //Set drive powers
            //drive.setDrivePowers(drivePower);

            //drive.updatePoseEstimate();

            //telemetry.addData("Axial (Forward/Back)", axial);
            //telemetry.addData("Lateral (Strafe)", lateral);
            //telemetry.addData("Heading (Turn)", heading);
            telemetry.addData("Launch speed", launchMotor.getPower());
            telemetry.addData("Launch speed", intakeMotor.getPower());
            telemetry.addData("is intake running?", isIntakeRunning);
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

    }
