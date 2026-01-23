package org.firstinspires.ftc.teamcode.part;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import static org.firstinspires.ftc.teamcode.part.Constants.*;
//import static org.firstinspires.ftc.teamcode.part.Constants.DRIVE_POWER;

public class Drive implements Part{



    public DcMotorEx motorLF, motorLR, motorRF, motorRR;


    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        TelemetrySystem.init(telemetry);
        motorLF = hardwareMap.get(DcMotorEx.class, "LF");
        motorLR = hardwareMap.get(DcMotorEx.class, "LR");
        motorRF = hardwareMap.get(DcMotorEx.class, "RF");
        motorRR = hardwareMap.get(DcMotorEx.class, "RR");
    }

    @Override
    public void start() {
        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void update() {
        return;
    }

    @Override
    public void stop() {
        motorLF.setPower(0);
        motorLR.setPower(0);
        motorRF.setPower(0);
        motorRR.setPower(0);
    }

}
