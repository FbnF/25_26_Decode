package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="M1",group="TeleOp")
public class Motor extends LinearOpMode {
    DcMotor my_Frontmotor;

    @Override
    public void runOpMode(){
        my_Frontmotor=hardwareMap.get(DcMotor.class,"frontMotor");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a) {
                my_Frontmotor.setPower(1);
            }
            else if (gamepad1.b){
                my_Frontmotor.setPower(-1);
            }
        }
    }
}
