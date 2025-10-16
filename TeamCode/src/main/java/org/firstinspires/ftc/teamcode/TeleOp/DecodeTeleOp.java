package org.firstinspires.ftc.teamcode.TeleOp;

// Ensure all necessary imports are present (from the sample code)
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive; // Make sure this import is correct for your project

@TeleOp(group = "DecodeTeleop")
public class DecodeTeleOp extends LinearOpMode{
    @Override
    public void runOpMode() {
        // Initialize the drive class
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0, 0, 0));
        DcMotor launchMotor;
        double speedFactor = 0.5;
        // Wait for the start button to be pressed
        waitForStart();

        // Ensure the op mode is active and the robot is not interrupted
        while (opModeIsActive()) {
            // Get input from the gamepad
            double axial = -gamepad1.right_stick_y * speedFactor; // Invert the y-axis
            double lateral = gamepad1.left_stick_x * speedFactor;  // Strafe is x-axis
            double heading = gamepad1.right_stick_x * speedFactor; // Rotation is x-axis of right stick

            PoseVelocity2d drivePower = new PoseVelocity2d(
                    new Vector2d(
                            axial,
                            lateral
                    ),
                    heading

            );
            //Set drive powers
            drive.setDrivePowers(drivePower);

            drive.updatePoseEstimate();
            telemetry.addData("Axial (Forward/Back)", axial);
            telemetry.addData("Lateral (Strafe)", lateral);
            telemetry.addData("Heading (Turn)", heading);
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
