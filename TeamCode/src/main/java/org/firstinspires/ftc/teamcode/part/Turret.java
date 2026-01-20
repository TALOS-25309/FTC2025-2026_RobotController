package org.firstinspires.ftc.teamcode.part;

import static org.firstinspires.ftc.teamcode.part.Constants.*;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.PID;


public class Turret implements Part {

    DcMotorEx motor;
    Encoder encoder;
    PID pidController;

    boolean PIDActivated;

    Vision vision;

    public Turret(Vision vision){
        this.vision = vision;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        encoder = hardwareMap.get(Encoder.class, "turret_motor");

        pidController = new PID(P,I,D);
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
        double position = encoder.getPositionAndVelocity().position;
        if (PIDActivated) {
            runPID();
            if (position < TURRET_LEFT_LIMIT){
                PIDActivated = false;
                setPosition((int)(position + ONE_REV_TICKS));
            }
            else if (position > TURRET_RIGHT_LIMIT){
                PIDActivated = false;
                setPosition((int)position - ONE_REV_TICKS);
            }
        }
        else {
            if (position < TURRET_LEFT_END
                    || position > TURRET_RIGHT_END){
                PIDActivated = true;
            }
        }
    }

    @Override
    public void stop() {
        PIDActivated = false;
        motor.setPower(0);
    }

    public void runPID(){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidController.updatePID(P,I,D);
        double currentAngle = vision.getPos()[0];
        double targetAngle = 0;
        double error = currentAngle - targetAngle;
        double pidOutput = pidController.update(error, -MOTOR_MAXIMUM_POWER, MOTOR_MAXIMUM_POWER);
        motor.setPower(pidOutput);
    }
    public void setPosition(int pos){
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
