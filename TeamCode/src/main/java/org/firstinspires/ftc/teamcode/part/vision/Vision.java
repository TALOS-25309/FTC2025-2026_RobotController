package org.firstinspires.ftc.teamcode.part.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.part.vision.VisionConst.*;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Vision {
    private  Limelight3A LL;
    private double x,y,z;
    private boolean detected;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        LL = hardwareMap.get(Limelight3A.class, "Limelight");
    }
    public void start() {;
        LL.setPollRateHz(250);
        LL.start();
    }
    public void setPipeline(PIPELINE pipeline) {
        switch (pipeline){
            case RED_GOAL:
                LL.pipelineSwitch(VisionConst.RED_PIPELINE);
                break;
            case BLUE_GOAL:
                LL.pipelineSwitch(VisionConst.BLUE_PIPELINE);
                break;
            case OBELISK_DETECTION:
                LL.pipelineSwitch(VisionConst.OBELISK_PIPELINE);
                break;
            default:
                LL.pipelineSwitch(VisionConst.RED_PIPELINE);
                break;
        }
    }

    public List<FiducialResult> fiducials;
    public LLStatus status;
    public double time;

    public void update(){

        LLResult result = LL.getLatestResult();
        status = LL.getStatus();

        if (result != null && result.isValid()) {
            time = result.getTimestamp();

            fiducials = result.getFiducialResults();
            if (fiducials.isEmpty()){
                detected = false;
            }
            else {
                detected = true;
                FiducialResult fiduciary = fiducials.get(0);
                this.x = fiduciary.getTargetPoseCameraSpace().getPosition().x;
                this.y = fiduciary.getTargetPoseCameraSpace().getPosition().y;
                this.z = fiduciary.getTargetPoseCameraSpace().getPosition().z;

            }
        }
        else{
            detected = false;
            time = -1;
        }
    }

    public int getFiducialID(){
        return fiducials.get(0).getFiducialId();
    }

    public double getTimestamp(){
        return LL.getLatestResult().getTimestamp();
    }

    public boolean tagDetected(){
        return detected;
    }

    public double[] getPos(){
        return new double[]{x,y,z};
    }





    public void stop(){
        LL.stop();
    }


}
