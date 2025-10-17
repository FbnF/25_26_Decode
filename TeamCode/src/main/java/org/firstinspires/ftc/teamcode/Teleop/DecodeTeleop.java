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

@TeleOp(group = "DecodeTeleop")
public class DecodeTeleop extends LinearOpMode {
    private DcMotorEx LaunchMotor;
    private DcMotorEx IntakeMotor;
    private int PosPowReq =0;
    private int NegPowReq =0;
    private int ZeroPower =0;
    private int RampPosPowReq =0;
    private int RampNegPowReq =0;
    private int ReducePowerInd =0;
    private double MotorPowerSign =0;



    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        // - - - Set up dashboard telemetry - - - //
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        LaunchMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        LaunchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // use braking to slow the motor down faster
        LaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // use braking to slow the motor down faster
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                // Increase Reduce Power indicator count by 1
                ReducePowerInd = ReducePowerInd +1;
            } else {
                // Reset the Reduce Power indicator count to zero
                ReducePowerInd =0;
            }

            if (NegPowReq==1){
                LaunchMotor.setPower(-1.0);
            } else if(PosPowReq==1) {
                LaunchMotor.setPower(1.0);
            } else if(ZeroPower==1) {
                LaunchMotor.setPower(0);
            } else if(ReducePowerInd==1) {
                MotorPowerSign = Math.signum(LaunchMotor.getPower());
                LaunchMotor.setPower(
                        Math.min(Math.max(-1.0,LaunchMotor.getPower()-MotorPowerSign*0.1),1.0));
            }

            // right stick y controls the full range of power when it is absolute
            // value greater than 0.1
            if (Math.abs(gamepad1.right_stick_y)> 0.1 ) {
                NegPowReq=0;
                PosPowReq=0;
                ReducePowerInd =0;
                ZeroPower =0;
                LaunchMotor.setPower(gamepad1.right_stick_y);
            }   else if (PosPowReq==0 && NegPowReq==0) {
                LaunchMotor.setPower(0);

            }
            // IntakeMotor

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
                IntakeMotor.setPower(0);
            }

            if (RampNegPowReq==1){
                IntakeMotor.setPower(-0.5);
            }
            if(RampPosPowReq==1) {
                IntakeMotor.setPower(0.5);
            }

            // right stick y controls the full range of power when it is absolute
            // value greater than 0.1
            if (Math.abs(gamepad2.right_stick_y)> 0.1 ) {
                RampNegPowReq=0;
                RampPosPowReq=0;
                IntakeMotor.setPower(gamepad2.right_stick_y);
            }   else if (RampPosPowReq==0 && RampNegPowReq==0) {
                IntakeMotor.setPower(0);

            }


            // Launch Motor Info
            telemetry.addData("Current  Launch Motor Power: ",
                    "%.3f", LaunchMotor.getPower());
            telemetry.addData("Current  Launch Motor Speed: ", 
                    "%.3f",LaunchMotor.getVelocity());
            //Intake Motor Info
            telemetry.addData("Current  Intake Motor Power: ", 
                    "%.3f", IntakeMotor.getPower());
            telemetry.addData("Current  Intake Motor Speed: ", 
                    "%.3f", IntakeMotor.getVelocity());
            telemetry.update();
        }


    }


}