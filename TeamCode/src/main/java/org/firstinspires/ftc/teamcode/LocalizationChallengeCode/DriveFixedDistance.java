package org.firstinspires.ftc.teamcode.LocalizationChallengeCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive Fixed Distance (Optimized)", group = "Localization Challenge Code")
public class DriveFixedDistance extends LinearOpMode {
    DcMotor leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;

    // --- Configuration Constants ---
    // Target ticks to travel (Example: 1000 ticks)
    private static final int TARGET_TICKS = 1000;
    // Power applied to motors during the movement
    private static final double DRIVE_POWER = 0.30;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Map hardware
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftRearMotor   = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightRearMotor  = hardwareMap.get(DcMotor.class, "rightRearMotor");

        // 2. Set Motor Directions (Ensure positive power means forward for all)
        // Adjust this if your robot moves backward when power is positive.
        // For standard FTC configurations, right motors usually need reversing.
        // leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Example if needed
        // rightFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Example if needed

        // 3. Setup Encoders for RUN_TO_POSITION
        // Reset encoders first
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Then set the BRAKE behavior for crisp stops
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set the final run mode
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 4. Set Target Positions
        // Since we just reset the encoders, the current position is 0.
        // We set the target position relative to the starting position (0 + TARGET_TICKS).
        setMotorTargetPosition(TARGET_TICKS);

        telemetry.addData("Status", "Initialization Complete. Ready to Drive.");
        telemetry.addData("Target Ticks", TARGET_TICKS);
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // 5. Start Driving
        // Setting power starts the movement when in RUN_TO_POSITION mode.
        // The motor will use internal PID control to move toward the target.
        setMotorPower(DRIVE_POWER);

        // 6. Monitor and Wait for Completion
        // We loop while the OpMode is active AND any motor is busy trying to reach its target.
        while (opModeIsActive() && isBusy()) {
            // Display current progress
            telemetry.addData("Target", "%d Ticks", TARGET_TICKS);
            telemetry.addData("LF", "Pos: %d / Busy: %b", leftFrontMotor.getCurrentPosition(), leftFrontMotor.isBusy());
            telemetry.addData("RF", "Pos: %d / Busy: %b", rightFrontMotor.getCurrentPosition(), rightFrontMotor.isBusy());
            telemetry.addData("LR", "Pos: %d / Busy: %b", leftRearMotor.getCurrentPosition(), leftRearMotor.isBusy());
            telemetry.addData("RR", "Pos: %d / Busy: %b", rightRearMotor.getCurrentPosition(), rightRearMotor.isBusy());
            telemetry.update();
        }

        // 7. Stop motors and reset mode
        setMotorPower(0);
        // It's good practice to switch back to RUN_USING_ENCODER mode after a RUN_TO_POSITION move.
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Drive Complete.");
        telemetry.update();
        sleep(1000); // Pause for a second to show final telemetry
    }

    /** Helper method to set the same RunMode for all drive motors */
    private void setMotorMode(DcMotor.RunMode mode) {
        leftFrontMotor.setMode(mode);
        leftRearMotor.setMode(mode);
        rightFrontMotor.setMode(mode);
        rightRearMotor.setMode(mode);
    }

    /** Helper method to set the same ZeroPowerBehavior for all drive motors */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontMotor.setZeroPowerBehavior(behavior);
        leftRearMotor.setZeroPowerBehavior(behavior);
        rightFrontMotor.setZeroPowerBehavior(behavior);
        rightRearMotor.setZeroPowerBehavior(behavior);
    }

    /** Helper method to set the same target position for all drive motors */
    private void setMotorTargetPosition(int target) {
        // Setting the target position is RELATIVE to the mode. Since we reset,
        // this is the absolute tick count we want to reach.
        leftFrontMotor.setTargetPosition(target);
        leftRearMotor.setTargetPosition(target);
        rightFrontMotor.setTargetPosition(target);
        rightRearMotor.setTargetPosition(target);
    }

    /** Helper method to set the same power for all drive motors */
    private void setMotorPower(double power) {
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);
    }

    /** Helper method to check if any motor is busy moving to its target position */
    private boolean isBusy() {
        return leftFrontMotor.isBusy() || leftRearMotor.isBusy() || rightFrontMotor.isBusy() || rightRearMotor.isBusy();
    }
}
