package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class AutoShootFSM {

    public enum AutoShoot {
        IDLE,
        RAMP_UP,
        INTAKE,
        WAIT,
        RAMP_DOWN
    }

    private AutoShoot state = AutoShoot.IDLE;
    private double stateTimer = 0;

    private DcMotor rightintake;
    private DcMotor leftintake;
    private Servo rampLeft;
    private Servo rampRight;
    private DcMotorEx leftshooter;
    private DcMotorEx rightshooter;


    public void init(HardwareMap hardwareMap) {
        rightintake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftintake  = hardwareMap.get(DcMotor.class, "leftIntake");
        leftshooter  = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightshooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rampLeft = hardwareMap.get(Servo.class, "rampLeft");
        rampRight = hardwareMap.get(Servo.class, "rampRight");

        leftintake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightintake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftshooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rampLeft.setDirection(Servo.Direction.FORWARD);
        rampRight.setDirection(Servo.Direction.REVERSE);

        leftshooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightshooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }

    public void trigger(){
        if (state == AutoShoot.IDLE){
            state = AutoShoot.RAMP_UP;
        }
    }

    public void update (double currentTime){
        switch (state){

            case RAMP_UP:
                rampLeft.setPosition(0.22);
                rampRight.setPosition(0.22);
                stateTimer = currentTime;
                state = AutoShoot.INTAKE;
                break;

            case INTAKE:
                leftintake.setDirection(DcMotorSimple.Direction.FORWARD);
                rightintake.setDirection(DcMotorSimple.Direction.REVERSE);
                leftintake.setPower(1);
                rightintake.setPower(1);
                if (currentTime - stateTimer >= 1){
                    leftintake.setPower(0);
                    rightintake.setPower(0);
                    stateTimer = currentTime;
                    state = AutoShoot.WAIT;
                }
                break;

            case WAIT:
                if (currentTime - stateTimer >= 1){
                    state = AutoShoot.RAMP_DOWN;
                }
                break;

            case RAMP_DOWN:
                rampLeft.setPosition(0);
                rampRight.setPosition(0);
                leftshooter.setPower(0.3);
                rightshooter.setPower(0.3);
                state = AutoShoot.IDLE;
                break;

            case IDLE:
            default:
                break;

        }

    }

    public boolean isRunning () {
        return state != AutoShoot.IDLE;
    }

    public AutoShoot getState () {
        return state;
    }

}
