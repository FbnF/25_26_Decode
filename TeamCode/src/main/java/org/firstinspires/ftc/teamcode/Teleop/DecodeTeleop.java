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
    private DcMotorEx RampMotor;
    private int PosPowReq =0;
    private int NegPowReq =0;
    private int ZeroPower =0;
    private int RampPosPowReq =0;
    private int RampNegPowReq =0;
    private int ReducePowerInd =0;



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


        RampMotor = hardwareMap.get(DcMotorEx.class, "RampMotor");
        RampMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RampMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // - - - Waiting for start signal from driver station - - - //
        waitForStart();

        while (!isStopRequested()) {

            // - - - Mecanum drive control - - - //
            // button a to set power to 1.0
            if (gamepad1.a) {
                PosPowReq = 1;
                NegPowReq=0;
                ZeroPower =0;
                ReducePowerInd =0;
            }

            // button b to set power to -1.0
            if (gamepad1.b) {
                PosPowReq = 0;
                NegPowReq = 1;
                ZeroPower =0;
                ReducePowerInd =0;

            }

            //button x to set power to zero
            if (gamepad1.x) {
                NegPowReq=0;
                PosPowReq=0;
                ZeroPower=1;
                ReducePowerInd =0;
            }

            //button Y to reduce power by 0.1
            if(gamepad1.y) {
                NegPowReq=0;
                PosPowReq=0;
                ZeroPower=0;
                ReducePowerInd = ReducePowerInd +1;
            }

            if (NegPowReq==1){
                ArmMotor.setPower(-1.0);
            } else if(PosPowReq==1) {
                ArmMotor.setPower(1.0);
            } else if(ZeroPower==1) {
                ArmMotor.setPower(0);
            } else if(ReducePowerInd==1) {
                ArmMotor.setPower(Math.min(Math.max(-1.0,ArmMotor.getPower()-0.1),1.0));
            }

            // right stick y controls the full range of power when it is absolute
            // value greater than 0.1
            if (Math.abs(gamepad1.right_stick_y)> 0.1 ) {
                NegPowReq=0;
                PosPowReq=0;
                ArmMotor.setPower(gamepad1.right_stick_y);
            }   else if (PosPowReq==0 && NegPowReq==0) {
                ArmMotor.setPower(0);

            }
            // Rampmotor

            if (gamepad2.a) {
                RampPosPowReq = 1;
                RampNegPowReq=0;
            }

            // button b to set power to -1.0
            if (gamepad2.b) {
                RampPosPowReq = 0;
                RampNegPowReq = 1;

            }

            //button x to set power to zero
            if (gamepad2.x) {
                RampNegPowReq=0;
                RampPosPowReq=0;
                RampMotor.setPower(0);
            }

            if (RampNegPowReq==1){
                RampMotor.setPower(-1.0);
            }
            if(RampPosPowReq==1) {
                RampMotor.setPower(1.0);
            }

            // right stick y controls the full range of power when it is absolute
            // value greater than 0.1
            if (Math.abs(gamepad2.right_stick_y)> 0.1 ) {
                RampNegPowReq=0;
                RampPosPowReq=0;
                RampMotor.setPower(gamepad2.right_stick_y);
            }   else if (RampPosPowReq==0 && RampNegPowReq==0) {
                RampMotor.setPower(0);

            }



            telemetry.addData("Current  Power level: ", ArmMotor.getPower());
            telemetry.update();

            telemetry.addData("Current  Power level: ", RampMotor.getPower());
            telemetry.update();
        }


    }


}