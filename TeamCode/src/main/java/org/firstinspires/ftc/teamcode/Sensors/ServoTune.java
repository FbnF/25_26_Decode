package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name = "Sensor: ServoTune", group = "Sensor")
public class ServoTune extends LinearOpMode {

    private Servo test_servo;
    private boolean UpRequest,DownRequest;
    boolean prev1a = false, prev1b=false;


    @Override
    public void runOpMode() {
        test_servo = hardwareMap.get(Servo.class, "TestServo");
        test_servo.setPosition(0); // Initial position

        waitForStart();

        // Run continuously until stopped
        while (opModeIsActive()) {

//          Gamepad1 a button will increase the servo position by 0.01
            UpRequest = gamepad1.a && !prev1a;
            if(UpRequest) {
            test_servo.setPosition(test_servo.getPosition()+0.01);
            }
            prev1a= gamepad1.a;

//          Gamepad1 b button will decrease the servo position by 0.01
            DownRequest = gamepad1.b && !prev1b;
            if(DownRequest) {
                test_servo.setPosition(test_servo.getPosition()-0.01);
            }
            prev1b= gamepad1.b;

            telemetry.addData("Servo Position", "%.2f", test_servo.getPosition());
            telemetry.update();

        }
    }
}