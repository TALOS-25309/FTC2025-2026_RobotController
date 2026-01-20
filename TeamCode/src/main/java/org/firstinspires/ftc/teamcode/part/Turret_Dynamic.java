package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.PID;

import org.firstinspires.ftc.teamcode.part.Vision;


public class Turret_Dynamic implements Part
{
    // Turret motors & encoders
    DcMotorEx turretMotor;
    Encoder turretEncoder;

    // PID controller for turret.
    // Note, this same variable will also be used for updating @ roadrunner
    PID pidController;

    // This is to be updated real time @ limelight,
    // The value is in degrees.

    // Logic:
    // Roadrunner will tell us how 'turned' the robot is.
    // Initially, this will probably be 0, as well as turretEncoder value.
    // So, we target -roadrunner when we can't see the april tag.
    // But as time go by, roadrunner value error will cumulate, eventually being non-trustable.
    // So, when we see the april tag, we update this value.
    // We will target -roadrunner + roadrunner_turretEncoder_offset.
    // We never update pid with raw limelight -- we will modify the offset to do that.
    double roadrunner_turretEncoder_offset = 0.0;
    boolean offsetEverUpdated = false;

    Vision limelight;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

    }
}
