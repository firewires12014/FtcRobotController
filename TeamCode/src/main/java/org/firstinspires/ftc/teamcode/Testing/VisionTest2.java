package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.CenterstageDetectorBlue;
import org.firstinspires.ftc.teamcode.Vision.CenterstageDetectorRed;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Vision Test 2", group="FireWires")
public class VisionTest2 extends OpMode {
    public OpenCvCamera camera;
    private CenterstageDetectorRed CenterstageDetectorRed = new CenterstageDetectorRed(telemetry);


    @Override
    public void init() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(CenterstageDetectorRed);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addData("Status:", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {

        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        CenterstageDetectorRed.Location location = CenterstageDetectorRed.getLocation();
        telemetry.addData("Location: ", location);
        telemetry.update();

        
    }

}