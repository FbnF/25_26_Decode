package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(name = "Sensor: LED Light Strip", group = "Sensor")
public class LEDLightStrip extends LinearOpMode {

    private Servo blinkin;
    double blinkinValue;
    double servoPosition;
    double step;


    @Override
    public void runOpMode(){
//        blinkin = hardwareMap.get(Servo.class, "blinkin");

        RevBlinkinLedDriver blinkin;
        RevBlinkinLedDriver.BlinkinPattern preInit = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
        RevBlinkinLedDriver.BlinkinPattern main = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
        RevBlinkinLedDriver.BlinkinPattern endgame = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        RevBlinkinLedDriver.BlinkinPattern climbAlert = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        RevBlinkinLedDriver.BlinkinPattern AlertTest = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        blinkin.setPattern(preInit);
        

        waitForStart();

        while (opModeIsActive()) {
            blinkin.setPattern(main);
            sleep(5000);
            blinkin.setPattern(climbAlert);
            sleep(5000);
            blinkin.setPattern(endgame);
            sleep(5000);
            blinkin.setPattern(AlertTest);
            sleep(5000);
//
//      }

        }
    }
}
