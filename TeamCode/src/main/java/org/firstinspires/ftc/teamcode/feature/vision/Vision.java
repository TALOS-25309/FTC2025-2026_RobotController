package org.firstinspires.ftc.teamcode.feature.vision;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.part.Constants.*;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;

import java.util.List;

public class Vision {
    private  Limelight3A LL;

    private double angle, distance;
    private boolean detected;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        LL = hardwareMap.get(Limelight3A.class, "Limelight");
    }
    public void start() {;
        LL.setPollRateHz(40);
        LL.start();

        detected = false;
        distance = 0;
    }
    public void setPipeline(PIPELINE pipeline) {
        switch (pipeline){
            case RED_GOAL:
                LL.pipelineSwitch(RED_PIPELINE);
                break;
            case BLUE_GOAL:
                LL.pipelineSwitch(BLUE_PIPELINE);
                break;
            case OBELISK_DETECTION:
                LL.pipelineSwitch(OBELISK_PIPELINE);
                break;
            default:
                LL.pipelineSwitch(RED_PIPELINE);
                break;
        }
    }

    public List<FiducialResult> fiducials;
//    public LLStatus status;
    public double time;


    public void update(){

        LLResult result = LL.getLatestResult();
//        status = LL.getStatus();


        if (result != null && result.isValid()) {
            time = result.getTimestamp();

            fiducials = result.getFiducialResults();
            if (fiducials.isEmpty()){
                detected = false;
            }
            else {
                detected = true;
                FiducialResult fiduciary = fiducials.get(0);

                angle = result.getTx();

                distance = Math.pow(
                            Math.pow(fiduciary.getTargetPoseCameraSpace().getPosition().z, 2)
                                + Math.pow(fiduciary.getTargetPoseCameraSpace().getPosition().x, 2),
                            0.5
                        );
                TelemetrySystem.addClassData("Vision","x",fiduciary.getTargetPoseRobotSpace().getPosition().x);
                TelemetrySystem.addClassData("Vision","z",fiduciary.getTargetPoseRobotSpace().getPosition().z);

            }

        }
        else{
            detected = false;
            time = -1;
        }


        TelemetrySystem.addClassData("Vision","Timestamp", time);
    }

    public int getFiducialID(){
        return fiducials.get(0).getFiducialId();
    }

    // Warning : getting the status of Limelight takes a lot of time.
    // To provide enough working rate, remove this method in the main loop.
    public LLStatus getStatus(){
        return LL.getStatus();
    }

    public boolean tagDetected(){
        return detected;
    }

    public double getDistance(){
        return distance;
    }

    public double getAngle(){
        return angle;
    }



    public void stop(){
        LL.stop();
    }


}
