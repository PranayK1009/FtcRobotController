package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop")
public class APOC_TeleOp extends OpMode {

    private Limelight3A limelight;
    private final TURRETMECHANISM turret = new TURRETMECHANISM();
    private final AutoShootFSM autoShootFSM = new AutoShootFSM();

    private Servo liftRight, liftLeft, Hservo;
    private DcMotorEx leftshooter, rightshooter;
    private DcMotor leftintake, rightintake;
    private Servo rampLeft, rampRight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private double hoodPosition = 0.0;
    private double shooterPower = 0.3;

    private boolean prevB           = false;
    private boolean prevX           = false;
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper  = false;
    private boolean prevY           = false;
    private boolean rampDeployed    = false;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        turret.init(hardwareMap);
        autoShootFSM.init(hardwareMap);

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        liftRight = hardwareMap.get(Servo.class, "rightLift");
        liftLeft  = hardwareMap.get(Servo.class, "leftLift");
        liftRight.setDirection(Servo.Direction.FORWARD);
        liftLeft.setDirection(Servo.Direction.REVERSE);

        leftshooter  = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightshooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftintake   = hardwareMap.get(DcMotor.class, "leftIntake");
        rightintake  = hardwareMap.get(DcMotor.class, "rightIntake");
        rampLeft     = hardwareMap.get(Servo.class, "rampLeft");
        rampRight    = hardwareMap.get(Servo.class, "rampRight");
        leftshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Hservo = hardwareMap.get(Servo.class, "hoodServo");
        Hservo.setDirection(Servo.Direction.REVERSE);
        Hservo.setPosition(hoodPosition);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        telemetry.addLine("Initialized");
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
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (gamepad1.dpad_down) imu.resetYaw();

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
        double rotY =  x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeft.setPower((rotY + rotX + rx) / denominator);
        backLeft.setPower((rotY - rotX + rx) / denominator);
        frontRight.setPower((rotY - rotX - rx) / denominator);
        backRight.setPower((rotY + rotX - rx) / denominator);

        // --- RAMP TOGGLE ---
        if (gamepad1.y && !prevY) rampDeployed = !rampDeployed;
        if (!autoShootFSM.isRunning()) {
            rampLeft.setPosition(rampDeployed ? 0.22 : 0);
            rampRight.setPosition(rampDeployed ? 0.22 : 0);
        }

        // --- INTAKE ---
        if (!autoShootFSM.isRunning()) {
            if (gamepad2.right_trigger > 0.1) {
                leftintake.setDirection(DcMotorSimple.Direction.FORWARD);
                rightintake.setDirection(DcMotorSimple.Direction.REVERSE);
                leftintake.setPower(1.0);
                rightintake.setPower(1.0);
            } else if (gamepad1.right_trigger > 0.1) {
                leftintake.setDirection(DcMotorSimple.Direction.FORWARD);
                rightintake.setDirection(DcMotorSimple.Direction.REVERSE);
                leftintake.setPower(0.7);
                rightintake.setPower(0.7);
            } else if (gamepad1.left_trigger > 0.1) {
                leftintake.setDirection(DcMotorSimple.Direction.REVERSE);
                rightintake.setDirection(DcMotorSimple.Direction.FORWARD);
                leftintake.setPower(0.7);
                rightintake.setPower(0.7);
            } else {
                leftintake.setPower(0);
                rightintake.setPower(0);
            }
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
        if (!autoShootFSM.isRunning()) {
            // Left bumper rising-edge debounce toggle logic
            if (gamepad2.left_bumper && !prevLeftBumper) {
                if (shooterPower == 0.3) {
                    shooterPower = 0.8;
                } else {
                    shooterPower = 0.3;
                }
                leftshooter.setPower(shooterPower);
                rightshooter.setPower(shooterPower);
            }

            // Retained 'Y' button as an explicit safety kill-switch
            if (gamepad2.y) {
                shooterPower = 0.0;
                leftshooter.setPower(shooterPower);
                rightshooter.setPower(shooterPower);
            }
        }

        // --- LIFT ---
        if (gamepad2.dpad_left) {
            liftLeft.setPosition(0.5);
            liftRight.setPosition(0.5);
        }
        if (gamepad2.dpad_right) {
            liftLeft.setPosition(0);
            liftRight.setPosition(0);
        }

        // --- AUTO SHOOT FSM (gamepad2 right bumper) ---
        if (gamepad2.right_bumper && !prevRightBumper && !autoShootFSM.isRunning()) {
            autoShootFSM.trigger();
        }
        autoShootFSM.update(getRuntime());

        // --- LIMELIGHT ---
        LLResult result = limelight.getLatestResult();
        boolean targetFound = false;
        double tx = 0;

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == 24) {
                    targetFound = true;
                    tx = result.getTx();
                    break;
                }
            }
        }

        turret.update(targetFound, tx);

        // --- DEBOUNCE ---
        prevB           = gamepad2.b;
        prevX           = gamepad2.x;
        prevRightBumper = gamepad2.right_bumper;
        prevLeftBumper  = gamepad2.left_bumper;
        prevY           = gamepad1.y;

        // --- TELEMETRY ---
        telemetry.addData("Target", targetFound);
        telemetry.addData("TX", "%.2f", tx);
        telemetry.addData("kP", "%.5f", turret.getkP());
        telemetry.addData("kD", "%.5f", turret.getkD());
        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(botHeading)));
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Left Shooter", leftshooter.getPower());
        telemetry.addData("Right Shooter", rightshooter.getPower());
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addLine("=== FSM ===");
        telemetry.addData("Shoot State", autoShootFSM.getState());
        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Left Intake", leftintake.getPower());
        telemetry.addData("Right Intake", rightintake.getPower());
        telemetry.addLine("=== SERVOS ===");
        telemetry.addData("RampLeft", rampLeft.getPosition());
        telemetry.addData("RampRight", rampRight.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}