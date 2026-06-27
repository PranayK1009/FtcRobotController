package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Basic Teleop")
public class Basic_Teleop extends OpMode {

    private DcMotor rightintake;
    private DcMotor leftintake;
    private Servo Rservo1;
    private Servo Rservo2;
    private Servo servo1;
    private Servo servo2;
    private Servo Hservo;
    private DcMotor leftshooter;
    private DcMotor rightshooter;
    private IMU imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double hoodPosition = 0.0;
    private double shooterPower = 0.0;

    // Debounce states
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;

    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        rightintake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftintake = hardwareMap.get(DcMotor.class, "leftIntake");

        leftshooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightshooter = hardwareMap.get(DcMotor.class, "rightShooter");
        Hservo = hardwareMap.get(Servo.class, "HoodServo");

        leftshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightshooter.setDirection(DcMotorSimple.Direction.REVERSE);

        Rservo1 = hardwareMap.get(Servo.class, "Rservo1");
        Rservo2 = hardwareMap.get(Servo.class, "Rservo2");

        leftintake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightintake.setDirection(DcMotorSimple.Direction.REVERSE);

        Rservo1.setDirection(Servo.Direction.REVERSE);
        Rservo2.setDirection(Servo.Direction.FORWARD);

        Rservo1.setPosition(0);
        Rservo2.setPosition(0);

        servo1.setPosition(0);
        servo2.setPosition(0);

        // Zero the hood servo on init
        hoodPosition = 0.0;
        Hservo.setPosition(hoodPosition);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);

        telemetry.addLine("IMU Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

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

        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // Rservos
        if (gamepad1.y) {
            Rservo1.setPosition(0.33);
            Rservo2.setPosition(0.33);
        }
        if (gamepad1.a) {
            Rservo1.setPosition(0);
            Rservo2.setPosition(0);
        }

        // Intake
        leftintake.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
        rightintake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        // Hood servo: B increases, X decreases, clamped to [0, 0.3]
        if (gamepad1.b && !prevB) {
            hoodPosition = Math.min(hoodPosition + 0.1, 0.3);
            Hservo.setPosition(hoodPosition);
        }
        if (gamepad1.x && !prevX) {
            hoodPosition = Math.max(hoodPosition - 0.1, 0.0);
            Hservo.setPosition(hoodPosition);
        }

        // Shooter: right bumper increases power in 0.1 steps starting from 0.5,
        // left bumper turns off
        if (gamepad1.right_bumper && !prevRightBumper) {
            if (shooterPower == 0.0) {
                shooterPower = 0.5;
            } else {
                shooterPower = Math.min(shooterPower + 0.1, 1.0);
            }
            leftshooter.setPower(shooterPower);
            rightshooter.setPower(shooterPower);
        }
        if (gamepad1.left_bumper && !prevLeftBumper) {
            shooterPower = 0.0;
            leftshooter.setPower(0);
            rightshooter.setPower(0);
        }

        // Save button states for debouncing
        prevB            = gamepad1.b;
        prevX            = gamepad1.x;
        prevRightBumper  = gamepad1.right_bumper;
        prevLeftBumper   = gamepad1.left_bumper;

        telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
        telemetry.addData("Left Intake Speed", leftintake.getPower());
        telemetry.addData("Right Intake Speed", rightintake.getPower());
        telemetry.addData("RServo1 Position", Rservo1.getPosition());
        telemetry.addData("RServo2 Position", Rservo2.getPosition());
        telemetry.addData("Servo1", servo1.getPosition());
        telemetry.addData("Servo2", servo2.getPosition());
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Left Shooter", leftshooter.getPower());
        telemetry.addData("Right Shooter", rightshooter.getPower());
        telemetry.addData("Hood Servo Position", hoodPosition);
        telemetry.update();
    }
}