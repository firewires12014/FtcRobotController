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
        Pose2d startingPose = new Pose2d(16,-63.6, Math.toRadians(270));
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(23,-30, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                addTemporalMarker(()->{ dropper.Drop(); }).
                waitSeconds(1)
                .lineToLinearHeading(new Pose2d(23,-37, Math.toRadians(270)))
                //.turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(60,-37, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{lift.moveLift(-.3f);})
                .waitSeconds(1)
                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{outtake.pivotEnding();})
                .waitSeconds(1)
                .addTemporalMarker(()->{outtake.releaseSecondary(); outtake.releaseMain();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50,-37, Math.toRadians(180)))
                .addTemporalMarker(()->{outtake.pivotStart(); lift.moveLift(0.3);})
                .lineToLinearHeading(new Pose2d(52, -55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(16,-29, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{ dropper.Drop(); }).
                waitSeconds(1)
                .lineToLinearHeading(new Pose2d(16, -36.5, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(57.75,-32, Math.toRadians(180)),
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
                .lineToLinearHeading(new Pose2d(50,-32, Math.toRadians(180)))
                .addTemporalMarker(()->{outtake.pivotStart(); lift.moveLift(0.30);})
                .lineToLinearHeading(new Pose2d(54, -55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //Left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)

                .setReversed(true)
                .splineTo(new Vector2d(8,-32), Math.toRadians(180),
                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{ dropper.Drop(); }).
                waitSeconds(1)
                .lineToLinearHeading(new Pose2d(15,-32, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              //  .lineToLinearHeading(new Pose2d(13,-35),
              //          SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(30,-35, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(55, -26, Math.toRadians(180)),
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
                .lineToLinearHeading(new Pose2d(50,-25, Math.toRadians(180)))
                .addTemporalMarker(()->{outtake.pivotStart(); lift.moveLift(0.3);})
                .lineToLinearHeading(new Pose2d(54, -61, Math.toRadians(180)),
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
                    drive.followTrajectorySequence(leftMovementOne);
                    sleep(300000);
                    break;

                case MIDDLE:
                    drive.followTrajectorySequence(middleMovementOne);
                    sleep(30000);
                    break;

                case RIGHT:
                    drive.followTrajectorySequence(rightMovementOne);
                    sleep(300000);
                    break;
            }


            sleep(30000);
        }

    }

}


