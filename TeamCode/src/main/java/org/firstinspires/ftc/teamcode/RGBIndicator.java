package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "RGBIndicator", group = "TeleOp")
public class RGBIndicator extends LinearOpMode {

    private Servo blinkin;

    // Blink control variables
    private final ElapsedTime blinkTimer = new ElapsedTime();
    private boolean blinking = false;
    private boolean blinkState = false;
    private int blinkCount = 0;

    // Tunables
    private static final int FLASH_AMOUNT = 10;
    private static final long BLINK_INTERVAL_MS = 200;

    @Override
    public void runOpMode() throws InterruptedException {

        blinkin = hardwareMap.get(Servo.class, "PuckLight");

        waitForStart();

        while (opModeIsActive()) {

            // --- Direct manual control ---
            if (gamepad1.a) {
                blinking = false;
                blinkin.setPosition(0.368);
            }
            if (gamepad1.b) {
                blinking = false;
                blinkin.setPosition(0.444);
            }
            if (gamepad1.x) {
                blinking = false;
                blinkin.setPosition(0.287);
            }

            // --- Trigger blinking sequence ---
            if (gamepad1.y && !blinking) {
                blinking = true;
                blinkState = false;
                blinkCount = 0;
                blinkTimer.reset();
            }

            // --- Non-blocking blink state machine ---
            if (blinking) {
                if (blinkTimer.milliseconds() >= BLINK_INTERVAL_MS) {
                    blinkTimer.reset();

                    blinkState = !blinkState;

                    if (blinkState) {
                        blinkin.setPosition(0.368);
                    } else {
                        blinkin.setPosition(0.0);
                        blinkCount++;
                    }

                    if (blinkCount >= FLASH_AMOUNT) {
                        blinking = false;
                        blinkin.setPosition(0.0);
                    }
                }
            }

            telemetry.addData("Blinking", blinking);
            telemetry.addData("Blink Count", blinkCount);
            telemetry.update();
        }
    }
}
