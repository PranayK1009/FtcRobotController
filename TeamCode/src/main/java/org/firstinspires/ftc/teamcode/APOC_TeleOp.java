package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop")
public class APOC_TeleOp extends OpMode {

    private Limelight3A limelight;

    // Turret
    private final TURRETMECHANISM turret =
            new TURRETMECHANISM();

    private DcMotor rightintake;
    private DcMotor leftintake;
    private Servo rampLeft;
    private Servo rampRight;
    private Servo liftRight;
    private Servo liftLeft;
    private Servo Hservo;
    private DcMotor leftshooter;
    private DcMotor rightshooter;

    private IMU imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double hoodPosition = 0.0;
    private double shooterPower = 0.3;

    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;

    private boolean prevy = true;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        turret.init(hardwareMap);

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Brake mode so robot stops sharply during testing
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        liftRight = hardwareMap.get(Servo.class, "rightLift");
        liftLeft = hardwareMap.get(Servo.class, "leftLift");

        liftRight.setDirection(Servo.Direction.FORWARD);
        liftLeft.setDirection(Servo.Direction.REVERSE);

        rightintake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftintake  = hardwareMap.get(DcMotor.class, "leftIntake");

        leftshooter  = hardwareMap.get(DcMotor.class, "leftShooter");
        rightshooter = hardwareMap.get(DcMotor.class, "rightShooter");
        Hservo       = hardwareMap.get(Servo.class, "hoodServo");

        leftshooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightshooter.setDirection(DcMotorSimple.Direction.FORWARD);


        rampLeft = hardwareMap.get(Servo.class, "rampLeft");
        rampRight = hardwareMap.get(Servo.class, "rampRight");

        leftintake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightintake.setDirection(DcMotorSimple.Direction.FORWARD);

        rampLeft.setDirection(Servo.Direction.FORWARD);
        rampRight.setDirection(Servo.Direction.REVERSE);

        rampLeft.setPosition(0);
        rampRight.setPosition(0);

        Hservo.setDirection(Servo.Direction.REVERSE);;;

        hoodPosition = 0.0;
        Hservo.setPosition(hoodPosition);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);

        telemetry.addLine("=== Test Teleop Initialized ===");
        telemetry.addLine("RB = shooter up | LB = shooter off");
        telemetry.addLine("B = hood up | X = hood down");
        telemetry.addLine("Y = rollers out | A = rollers in");
        telemetry.addLine("RS X-axis = turret | Dpad Down = reset IMU");
        telemetry.update();
    }

    @Override
    public void start() {
        leftshooter.setPower(shooterPower);
        rightshooter.setPower(shooterPower);
        turret.resetTimer();
    }

    @Override
    public void loop() {

        // --- DRIVE ---
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.dpad_down) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(
                Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx),
                1
        );

        frontLeft.setPower((rotY + rotX + rx) / denominator);
        backLeft.setPower((rotY - rotX + rx) / denominator);
        frontRight.setPower((rotY - rotX - rx) / denominator);
        backRight.setPower((rotY + rotX - rx) / denominator);


        // --- RAMP SERVOS ---
        if (gamepad1.y) {
            prevy = !prevy;
        }

        if(!prevy){
            rampLeft.setPosition(0.22);
            rampRight.setPosition(0.22);
        }
        else{
            rampRight.setPosition(0);
            rampLeft.setPosition(0);
        }



        // --- INTAKE ---
        if (gamepad1.right_trigger > 0.1) {
            leftintake.setDirection(DcMotorSimple.Direction.FORWARD);
            rightintake.setDirection(DcMotorSimple.Direction.REVERSE);
            leftintake.setPower(1.0);
            rightintake.setPower(1.0);
        } else if (gamepad1.left_trigger > 0.1) {
            leftintake.setDirection(DcMotorSimple.Direction.FORWARD);
            rightintake.setDirection(DcMotorSimple.Direction.REVERSE);
            leftintake.setPower(0.7);
            rightintake.setPower(0.7);
        } else {
            leftintake.setPower(0);
            rightintake.setPower(0);
        }


        if(gamepad1.right_bumper){
            leftintake.setDirection(DcMotorSimple.Direction.REVERSE);
            rightintake.setDirection(DcMotorSimple.Direction.FORWARD);
            leftintake.setPower(0.7);
            rightintake.setPower(0.7);

        }

        // --- HOOD ---
        if (gamepad2.b && !prevB) {
            hoodPosition = Math.min(hoodPosition + 0.1, 0.3);
            Hservo.setPosition(hoodPosition);
        }
        if (gamepad2.x && !prevX) {
            hoodPosition = Math.max(hoodPosition - 0.1, 0.0);
            Hservo.setPosition(hoodPosition);
        }

        // --- SHOOTER ---
        if (gamepad2.right_bumper && !prevRightBumper) {
            shooterPower = Math.min(shooterPower + 0.5, 1);
            leftshooter.setPower(shooterPower);
            rightshooter.setPower(shooterPower);
        }
        if (gamepad2.left_bumper && !prevLeftBumper) {
            leftshooter.setPower(0.3);
            rightshooter.setPower(0.3);
        }

        if(gamepad2.y){
            leftshooter.setPower(0);
            rightshooter.setPower(0);
        }

        if(gamepad2.dpad_left){
            liftLeft.setPosition(0.5);
            liftRight.setPosition(0.5);
        }
        if(gamepad2.dpad_right){
            liftLeft.setPosition(0);
            liftRight.setPosition(0);
        }


        LLResult result =
                limelight.getLatestResult();

        boolean targetFound = false;

        double tx = 0;


        if (result != null && result.isValid()) {

            for (LLResultTypes.FiducialResult tag :
                    result.getFiducialResults()) {

                if (tag.getFiducialId() == 24) {

                    targetFound = true;

                    tx = result.getTx();

                    break;
                }
            }
        }

        turret.update(
                targetFound,
                tx
        );


        telemetry.addData(
                "Target",
                targetFound
        );

        telemetry.addData(
                "TX",
                "%.2f",
                tx
        );

        telemetry.addData(
                "kP",
                "%.5f",
                turret.getkP()
        );

        telemetry.addData(
                "kD",
                "%.5f",
                turret.getkD()
        );



        // --- DEBOUNCE ---
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevRightBumper = gamepad1.right_bumper;
        prevLeftBumper = gamepad1.left_bumper;

        // --- TELEMETRY ---
        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(botHeading)));
        telemetry.addData("Stick Input (x/y/rx)", String.format("%.2f / %.2f / %.2f", x, y, rx));

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Left Shooter", leftshooter.getPower());
        telemetry.addData("Right Shooter", rightshooter.getPower());
        telemetry.addData("Hood Position", hoodPosition);

        telemetry.addLine("=== TURRET ===");

        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Left Intake", leftintake.getPower());
        telemetry.addData("Right Intake", rightintake.getPower());

        telemetry.addLine("=== SERVOS ===");
        telemetry.addData("RServo1", rampLeft.getPosition());
        telemetry.addData("RServo2", rampRight.getPosition());

        telemetry.update();

    }

    @Override
    public void stop() {

        if (limelight != null) {

            limelight.stop();
        }
    }
}