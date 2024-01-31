package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
        //intake.resetIntake();
        outtake.lockPixels();
        dropper.Hold();
        Pose2d startingPose = new Pose2d(16,-63.6, Math.toRadians(90));
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // LEFT
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(23,-35, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                addTemporalMarker(()->{ dropper.Drop(); }).
                waitSeconds(1)
                .lineToLinearHeading(new Pose2d(23,-45, Math.toRadians(90)))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(55,-43, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{lift.moveLift(-.3f);})
                .waitSeconds(1)
                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{outtake.pivotEnding();})
                .waitSeconds(1)
                .addTemporalMarker(()->{outtake.releaseSecondary(); outtake.releaseMain();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50,-43, Math.toRadians(180)))
                .addTemporalMarker(()->{outtake.pivotStart(); lift.moveLift(0.3);})
                .lineToLinearHeading(new Pose2d(47, -65, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(16,-31, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{ dropper.Drop(); }).
                waitSeconds(1)
                .lineToLinearHeading(new Pose2d(16, -45, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(52.75,-40, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{lift.moveLift(-.30f);})
                .waitSeconds(1)
                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{outtake.pivotEnding();})
                .waitSeconds(1)
                .addTemporalMarker(()->{outtake.releaseSecondary(); outtake.releaseMain();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50,-40, Math.toRadians(180)))
                .addTemporalMarker(()->{outtake.pivotStart(); lift.moveLift(0.30);})
                .lineToLinearHeading(new Pose2d(47, -65, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //RIGHT
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .splineTo(new Vector2d(7,-35), Math.toRadians(180),
                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{ dropper.Drop(); }).
                waitSeconds(1)
                .lineToLinearHeading(new Pose2d(13,-35),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(30,-35, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(56.5, -29, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{lift.moveLift(-.3f);})
                .waitSeconds(1)
                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{outtake.pivotEnding();})
                .waitSeconds(1)
                .addTemporalMarker(()->{outtake.releaseSecondary(); outtake.releaseMain();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50,-27, Math.toRadians(180)))
                .addTemporalMarker(()->{outtake.pivotStart(); lift.moveLift(0.3);})
                .lineToLinearHeading(new Pose2d(47, -65, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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


            VisionRedClose.Location location = VisionRedClose.getLocation();


            switch (location) {
                case LEFT:
//                    drive.followTrajectorySequence(leftMovementOne);
                    //lift.moveLift(-.90f);
                    //sleep(200);
                    //lift.moveLift(0f);
                    //outtake.pivotEnding();
                    //sleep(1500);
                    //outtake.releaseSecondary();
                    break;
//                case MIDDLE:
//                    drive.followTrajectorySequence(middleMovementOne);
//                    break;

                case RIGHT:
                    drive.followTrajectorySequence(rightMovementOne);
                    break;
            }


            stop();
        }

    }

}


