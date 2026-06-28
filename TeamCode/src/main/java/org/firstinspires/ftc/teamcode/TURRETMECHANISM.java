package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TURRETMECHANISM {

    private CRServo turretservo1;
    private CRServo turretservo2;


    // ---------------- PID VALUES ----------------

    private double kP = -0.02;
    private double kD = 0.0002;

    // Desired tx value
    private double goalX = 0;

    // Previous error for derivative
    private double lastError = 0;

    // Deadband
    private double angleTolerance = 0.2;

    // Max turret power
    private final double MAX_POWER = 0.6;

    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();


    // ------------------------------------------------

    public void init(HardwareMap hwMap) {

       // turret = hwMap.get(DcMotorEx.class, "turret");
        turretservo1 = hwMap.get(CRServo.class, "rightTurret");
        turretservo2 = hwMap.get(CRServo.class, "leftTurret");

        turretservo1.setDirection(DcMotorSimple.Direction.REVERSE);

        timer.reset();
    }

    // ---------------- KP ----------------

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }

    // ---------------- KD ----------------

    public void setkD(double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }

    // ---------------- TIMER ----------------

    public void resetTimer() {
        timer.reset();
    }

    // ====================================================
    // MAIN UPDATE FUNCTION
    // ====================================================

    public void update(boolean targetFound, double tx) {

        double deltaTime = timer.seconds();
        timer.reset();

        // ------------------------------------------------
        // NO TARGET FOUND
        // ------------------------------------------------

        if (!targetFound) {
            turretservo1.setPower(0);
            turretservo2.setPower(0);


            lastError = 0;

            return;
        }

        // ------------------------------------------------
        // PD CONTROLLER
        // ------------------------------------------------

        // tx is Limelight horizontal offset
        // goal is tx = 0

        double error = goalX - tx;

        // Proportional
        double pTerm = error * kP;

        // Derivative
        double dTerm = 0;

        if (deltaTime > 0) {

            dTerm =
                    ((error - lastError) / deltaTime)
                            * kD;
        }

        // ------------------------------------------------
        // TOLERANCE CHECK
        // ------------------------------------------------

        if (Math.abs(error) < angleTolerance) {

            power = 0;

        } else {

            power = Range.clip(
                    pTerm + dTerm,
                    -MAX_POWER,
                    MAX_POWER
            );
        }

        // ------------------------------------------------
        // APPLY POWER
        // ------------------------------------------------


        lastError = error;
    }
    public void setservos (double power){
        turretservo1.setPower(power);
        turretservo2.setPower(power);
    }
}