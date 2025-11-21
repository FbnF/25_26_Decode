package org.firstinspires.ftc.teamcode.Autonomous;

// --- Roadrunner Libraries ---

// --- FTC Libraries ---
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

// -- Defined by us ---
import org.firstinspires.ftc.teamcode.MainCode.config.TagConfig;
import org.firstinspires.ftc.teamcode.MainCode.vision.AprilTagService;

// --- Data Logging ---
//import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLoggerForMotorTest;
import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLoggerFlex;

@TeleOp(name = "MotorTest", group = "TeleOp")
public class MotorTest extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx MotorWithVoltComp;
    private DcMotorEx MotorNoVoltComp;

    double ADAPTIVE_POWER;

    private VoltageSensor battery;
    // --- Vision ---
    private AprilTagService tagService;
    private boolean visionEnabled = false; // allows camera to be toggled on/off

    // Auto shooter (closed-loop velocity) path
    private boolean autoShooter = false;
    private boolean prevDpadUp = false, prevDpadDown = false;
    private double shooterSetpointTPS = 0.0;
    private static final double NO_SETPOINT = 0.0;

    // --- Config flags ---
    private static final boolean LOG_ENABLED = true;  // turn CSV logging on/off
    private TinyCsvLoggerFlex logger; // logging Data
    final double MAX_VOLTAGE = 12.5;
    double CURRENT_VOLTAGE = 0.0;
    boolean drivePrevRB = false, drivePrevLB = false;

    @Override
    public void runOpMode() {
        // Map hardware
        MotorWithVoltComp = hardwareMap.get(DcMotorEx.class,"Motor1");
        MotorNoVoltComp = hardwareMap.get(DcMotorEx.class,"Motor2");
        battery     = hardwareMap.voltageSensor.iterator().next();

        MotorWithVoltComp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorNoVoltComp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorNoVoltComp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // intake runs open-loop (no encoder feedback)
        MotorWithVoltComp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive (verify your constructor signature)

        // Vision
        tagService = new AprilTagService();
        //tagService.start(hardwareMap);

        // LOG: create CSV logger
        if (LOG_ENABLED) {
            logger = TinyCsvLoggerFlex.create(
                    hardwareMap,
                    "Voltage Motor Test",
                    TinyCsvLoggerFlex.motorEx("No Voltage Comp", MotorNoVoltComp),
                    TinyCsvLoggerFlex.doubleCol("Commanded No Volt", () -> 0.5),
                    TinyCsvLoggerFlex.motorEx("Voltage Comp", MotorWithVoltComp),
                    TinyCsvLoggerFlex.doubleCol("Voltage Comped power", () -> ADAPTIVE_POWER)




            );
        }


        waitForStart();

        // Safe startup
        MotorWithVoltComp.setPower(0.0);
        MotorNoVoltComp.setPower(0.0);

        while (opModeIsActive()) {
            CURRENT_VOLTAGE = battery.getVoltage();
            ADAPTIVE_POWER = 0.5 - ((CURRENT_VOLTAGE - MAX_VOLTAGE) * 0.05);
            MotorWithVoltComp.setPower(ADAPTIVE_POWER);
            MotorNoVoltComp.setPower(0.5);

            if (LOG_ENABLED && logger != null) {
                logger.record("run");
            }
        }


    }
}

