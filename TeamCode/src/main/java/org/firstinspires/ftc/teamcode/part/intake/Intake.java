package org.firstinspires.ftc.teamcode.part.intake;

import static org.firstinspires.ftc.teamcode.part.Constants.*;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.part.Part;



public class Intake implements Part {
    DcMotorEx motor;
    ColorSensor colorSensor;
    public IntakeState state;



    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "intake_motor");
//        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        state = IntakeState.IDLE;
    }

    @Override
    public void start() {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        colorSensor.enableLed(true);
    }

    @Override
    public void update() {
        if (this.state == IntakeState.RUN){
            this.cmdRun();
        }
    }

    @Override
    public void stop() {
        cmdStop();
    }


    // Commands
    public void cmdRun(){
        motor.setPower(INTAKE_POWER);
        state = IntakeState.RUN;
    }
    public void cmdStop(){
        motor.setPower(0);
        state = IntakeState.IDLE;
    }
    public void cmdReverseRun(){
        motor.setPower(-INTAKE_REVERSE_POWER);
        state = IntakeState.REVERSE_RUN;
    }
}
