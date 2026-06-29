package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTunerTutorial extends OpMode {

    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private double highVelocity = 1500;
    private double lowVelocity = 900;
    private double curTargetVelocity = highVelocity;
    private double F = 0;
    private double P = 0;
    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    private int stepIndex = 1;

    // --- DEBOUNCE TRACKERS ---
    private boolean prevY         = false;
    private boolean prevB         = false;
    private boolean prevDpadLeft  = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadUp    = false;
    private boolean prevDpadDown  = false;

    @Override
    public void init() {
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
        rightShooter.setDirection(DcMotorEx.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        telemetry.addLine("init complete");
    }

    @Override
    public void loop() {
        // --- BUTTON INPUTS WITH RISING-EDGE DETECTION ---

        // Toggle target velocity
        if (gamepad1.y && !prevY) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }

        // Change increment step size
        if (gamepad1.b && !prevB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Adjust F coefficient
        if (gamepad1.dpad_left && !prevDpadLeft)  F -= stepSizes[stepIndex];
        if (gamepad1.dpad_right && !prevDpadRight) F += stepSizes[stepIndex];

        // Adjust P coefficient
        if (gamepad1.dpad_up && !prevDpadUp)    P += stepSizes[stepIndex];
        if (gamepad1.dpad_down && !prevDpadDown)  P -= stepSizes[stepIndex];

        // --- HARDWARE UPDATES ---
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        leftShooter.setVelocity(curTargetVelocity);
        rightShooter.setVelocity(curTargetVelocity);

        // --- SAVE CURRENT STATES FOR NEXT LOOP ---
        prevY         = gamepad1.y;
        prevB         = gamepad1.b;
        prevDpadLeft  = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevDpadUp    = gamepad1.dpad_up;
        prevDpadDown  = gamepad1.dpad_down;

        // --- TELEMETRY ---
        double leftVelocity = leftShooter.getVelocity();
        double rightVelocity = rightShooter.getVelocity();

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Left Velocity", leftVelocity);
        telemetry.addData("Left Error", curTargetVelocity - leftVelocity);
        telemetry.addData("Right Velocity", rightVelocity);
        telemetry.addData("Right Error", curTargetVelocity - rightVelocity);
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.update();
    }
}