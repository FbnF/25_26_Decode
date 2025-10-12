package org.firstinspires.ftc.teamcode.Teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp(group = "Teleop")
public class DecodeTeleop extends LinearOpMode {
    private DcMotorEx ArmMotor;
    private int PosPowReq =0;
    private int NegPowReq =0;


    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        // - - - Set up dashboard telemetry - - - //
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // - - - Waiting for start signal from driver station - - - //
        waitForStart();

        while (!isStopRequested()) {

            // - - - Mecanum drive control - - - //
            // button a to set power to 1.0
            if (gamepad1.a) {
                PosPowReq = 1;
                NegPowReq=0;
            }

            // button b to set power to -1.0
            if (gamepad1.b) {
                PosPowReq = 0;
                NegPowReq = 1;

            }

            //button x to set power to zero
            if (gamepad1.x) {
                NegPowReq=0;
                PosPowReq=0;
                ArmMotor.setPower(0);
            }

            if (NegPowReq==1){
                ArmMotor.setPower(-1.0);
            }
            if(PosPowReq==1) {
                ArmMotor.setPower(1.0);
            }

            // right stick y controls the full range of power when it is absolute
            // value greater than 0.1
            if (Math.abs(gamepad1.right_stick_y)> 0.1 ) {
                NegPowReq=0;
                PosPowReq=0;
                ArmMotor.setPower(gamepad1.right_stick_y);
            }
            else{
                if (PosPowReq==0 && NegPowReq==0) {
                    ArmMotor.setPower(0);
                }
            }

            telemetry.addData("Current  Power level: ", ArmMotor.getPowerFloat());
            telemetry.update();
        }


    }


}