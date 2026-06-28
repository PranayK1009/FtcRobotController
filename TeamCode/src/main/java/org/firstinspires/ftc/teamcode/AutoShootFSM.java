package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class AutoShootFSM extends OpMode {

    public enum AutoShoot {
        IDLE,
        STORE,
        RAMP_UP,
        INTAKE,
        RAMP_DOWN,
        END
    }

    private DcMotor rightintake;
    private DcMotor leftintake;
    private Servo rampLeft;
    private Servo rampRight;
    private DcMotor leftshooter;
    private DcMotor rightshooter;

    @Override
    public void init() {
        rightintake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftintake  = hardwareMap.get(DcMotor.class, "leftIntake");
        leftshooter  = hardwareMap.get(DcMotor.class, "leftShooter");
        rightshooter = hardwareMap.get(DcMotor.class, "rightShooter");
        rampLeft = hardwareMap.get(Servo.class, "rampLeft");
        rampRight = hardwareMap.get(Servo.class, "rampRight");

        leftintake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightintake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftshooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rampLeft.setDirection(Servo.Direction.FORWARD);
        rampRight.setDirection(Servo.Direction.REVERSE);


    }

    @Override
    public void loop() {

    }
}
