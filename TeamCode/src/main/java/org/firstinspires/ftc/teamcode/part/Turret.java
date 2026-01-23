package org.firstinspires.ftc.teamcode.part;

import static org.firstinspires.ftc.teamcode.part.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.PID;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.part.vision.Vision;


public class Turret implements Part {

    DcMotorEx motor;
    DcMotorEx encoder;
    PID pidController;

    boolean PIDActivated;

    Vision vision;

    public Turret(Vision vision){
        this.vision = vision;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        encoder = hardwareMap.get(DcMotorEx.class, "turret");
//        encoder = motor;

        pidController = new PID(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    }

    @Override
    public void start() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDActivated = true;
//        RotationFinished = false;
    }


    @Override
    public void update() {
        double position = encoder.getCurrentPosition();
        TelemetrySystem.addClassData("TURRET", "position", position);
        TelemetrySystem.addClassData("TURRET", "PIDActivated", PIDActivated);
        if (PIDActivated){
            runPID();
        }
        else {
            motor.setPower(0);
        }

//        if (PIDActivated) {
//            runPID();
//            if (position < TURRET_LEFT_LIMIT){
//                PIDActivated = false;
//                setPosition((int)(position + ONE_REV_TICKS));
//            }
//            else if (position > TURRET_RIGHT_LIMIT){
//                PIDActivated = false;
//                setPosition((int)position - ONE_REV_TICKS);
//            }
//        }
//        else {
//            if (position < TURRET_LEFT_END
//                    || position > TURRET_RIGHT_END){
//                PIDActivated = true;
//            }
//        }
    }

    @Override
    public void stop() {
        PIDActivated = false;
        motor.setPower(0);
    }

    public void runPID(){
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidController.updatePID(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
        double currentAngle = vision.getPos()[0];
        TelemetrySystem.addClassData("VISION","timestamp", vision.getTimestamp());
        double targetAngle = 0;
        double error = currentAngle - targetAngle;
        TelemetrySystem.addClassData("TURRET", "error", error);
        double pidOutput = pidController.update(error, -TURRET_MAXIMUM_POWER, TURRET_MAXIMUM_POWER);
        TelemetrySystem.addClassData("TURRET", "pid", pidOutput);
        motor.setPower(pidOutput);

    }
    public void setPosition(int pos){
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
