package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.dropper;
import org.firstinspires.ftc.teamcode.Vision.VisionRedClose;
import org.firstinspires.ftc.teamcode.Vision.VisionRedFar;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "redClose", group = "Robot")

public class redClose extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionRedClose VisionRedClose = new VisionRedClose(telemetry); // camera stuff
    SampleMecanumDrive drive;
    Outtake outtake;
    dropper dropper;
    Lift lift;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(VisionRedClose); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        dropper = new dropper(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap);
        //intake.resetIntake();
        outtake.lockPixels();
        dropper.Hold();
        Pose2d startingPose = new Pose2d(16,-63.6, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(26, -26, Math.toRadians(180)),
                       SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                       SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.stack();})
                .lineToLinearHeading(new Pose2d(52, -44, Math.toRadians(180)),
                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //                .addTemporalMarker(()->{lift.moveLift(-.4f);})
//                .waitSeconds(1)
//                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->{outtake.pivotEnding();})
//                .waitSeconds(1)
//                .addTemporalMarker(()->{outtake.releaseSecondary(); outtake.releaseMain();})
//                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(50,-37, Math.toRadians(180)))
//                .addTemporalMarker(()->{outtake.pivotStart(); lift.moveLift(0.3);})
                .build();



        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(16, -22, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.stack();})
                .lineToLinearHeading(new Pose2d(50, -36, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.addTemporalMarker(()->{lift.moveLift(-.4f);})
//                .waitSeconds(1)
//                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->{outtake.pivotEnding();})
//                .waitSeconds(1)
//                .addTemporalMarker(()->{outtake.releaseSecondary(); outtake.releaseMain();})
//                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(50,-37, Math.toRadians(180)))
//                .addTemporalMarker(()->{outtake.pivotStart(); lift.moveLift(0.3);})
                .build();
        //Left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(12, -30, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.stack();})
                .lineToLinearHeading(new Pose2d(4, -30, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(50, -30, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                //.addTemporalMarker(()->{lift.moveLift(-.4f);})
//                .waitSeconds(1)
//                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->{outtake.pivotEnding();})
//                .waitSeconds(1)
//                .addTemporalMarker(()->{outtake.releaseSecondary(); outtake.releaseMain();})
//                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(50,-37, Math.toRadians(180)))
//                .addTemporalMarker(()->{outtake.pivotStart(); lift.moveLift(0.3);})


                .build();


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });
        while (!opModeIsActive()) {

            VisionRedClose.Location location = VisionRedClose.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }
        outtake.lockSecondary();
        outtake.releaseMain();
        waitForStart();

        while (opModeIsActive()) {
            intake.auto();
            sleep(1000);




            VisionRedClose.Location location = VisionRedClose.getLocation();

            switch (location) {
                case MIDDLE: //left
                    drive.followTrajectorySequence(leftMovementOne);
                    sleep(300000);
                    break;
//
//                case MIDDLE:
//                    drive.followTrajectorySequence(middleMovementOne);
//                    sleep(30000);
//                    break;

//                case MIDDLE: // right
//                    drive.followTrajectorySequence(rightMovementOne);
//                    sleep(300000);
//                    break;
            }


            sleep(30000);
        }

    }

}


