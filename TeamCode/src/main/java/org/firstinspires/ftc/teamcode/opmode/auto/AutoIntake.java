package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config("AutoIntake")
public class AutoIntake {
    DcMotorEx motor;
    public static double INTAKE_POWER = 1.0;
    public static double REVERSE_POWER = 0.4;

    AutoIntake(DcMotorEx intakeMotor){
        motor = intakeMotor;
    }

    void init() {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void intakeOn() {
        motor.setPower(INTAKE_POWER);
    }
    void intakeOff() {
        motor.setPower(0);
    }
    void intakeReverse() {
        motor.setPower(-1 * REVERSE_POWER);
    }
}
