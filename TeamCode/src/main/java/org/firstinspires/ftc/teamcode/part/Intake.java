package org.firstinspires.ftc.teamcode.part;

import static org.firstinspires.ftc.teamcode.part.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.part.Constants.INTAKE_REVERSE_POWER;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;



public class Intake implements Part{
    DcMotorEx motor;
    ColorSensor colorSensor;


    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
    }

    @Override
    public void start() {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        colorSensor.enableLed(true);
    }

    @Override
    public void update() {
        return;
    }

    @Override
    public void stop() {
        cmdStop();
    }


    // Commands
    public void cmdRun(){
        motor.setPower(INTAKE_POWER);
    }
    public void cmdStop(){
        motor.setPower(0);
    }
    public void cmdReverseRun(){
        motor.setPower(-INTAKE_REVERSE_POWER);
    }
}
