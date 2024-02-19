package org.firstinspires.ftc.teamcode.Auto;

import android.os.FileUriExposedException;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.dropper;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionBlueFar;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "blueFar", group = "Robot")
@Disabled
public class blueFar extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionBlueFar VisionBlueFar = new VisionBlueFar(telemetry); // camera stuff
    SampleMecanumDrive drive;
    Outtake outtake;
    Lift lift;
    Intake intake;
    dropper dropper;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(VisionBlueFar); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        dropper = new dropper(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        //intake.resetIntake();
        outtake.lockSecondary();
        outtake.releaseMain();
        Pose2d startingPose = new Pose2d(-39.5,63.6, Math.toRadians(90));
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // LEFT
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-39.5, 40, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-30,34), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{ dropper.Drop(); intake.stack(); }).
                waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-40,34, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-50,10, Math.toRadians(180)),
                         SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-58,13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.75)
                .addTemporalMarker(()->{ outtake.pivotAuto();intake.stack(); intake.in();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-53,13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{ intake.die(); intake.score(); intake.in();})
                .waitSeconds(0.55)
                .addTemporalMarker(()->{ lift.moveLift(0.05);intake.die();  outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(16, 12, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(60,20), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(57.5,33, Math.toRadians(180)), //white pixel drop
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{lift.moveLift(-.5f);})
                .waitSeconds(1)
                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{outtake.pivotEnding(); outtake.releaseMain();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ outtake.releaseMain();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(57.5,42, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{lift.moveLift(-.2f);})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.pivotEnding();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{outtake.releasePixels(); outtake.releaseSecondary();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(57,20, Math.toRadians(180)))
                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-34, 30,Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{dropper.Drop(); intake.stack();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-34,37, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-40,45,Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-50,8.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-57,8.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ outtake.pivotAuto();intake.stack(); intake.in();})
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(-53,8.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{ intake.die(); intake.score(); intake.in();})
                .waitSeconds(0.55)
                .addTemporalMarker(()->{ lift.moveLift(0.05);intake.die(); outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(16, 8.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(63,25), Math.toRadians(0),
        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{lift.moveLift(-.5f);})
                .waitSeconds(1)
                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{outtake.pivotEnding(); outtake.releaseMain();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ outtake.releaseMain();})
                .waitSeconds(0.75)
                .lineToLinearHeading(new Pose2d(60,25, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(61,32, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addTemporalMarker(()->{lift.moveLift(0.);lift.moveLift(-0.01f);})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.pivotEnding(); outtake.releaseMain();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{ outtake.releasePixels();})
                .waitSeconds(0.55)
                .addTemporalMarker(()-> {outtake.pivotStart(); lift.moveLift(0.3);})
                .lineToLinearHeading(new Pose2d(57,15, Math.toRadians(180)))
                .build();

        //Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-50,25, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{dropper.Drop(); intake.stack();})
                .lineToLinearHeading(new Pose2d(-53,25, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-53,8.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ outtake.pivotAuto(); intake.in();})
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(-48,8.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{ intake.die(); intake.score();intake.in();})
                .waitSeconds(0.55)
                .addTemporalMarker(()->{ lift.moveLift(0.05);intake.die(); outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(25, 8.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(60,39, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{lift.moveLift(-.5f);})
                .waitSeconds(1)
                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{outtake.pivotEnding(); outtake.releaseMain();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ outtake.releaseMain();})
                .waitSeconds(0.75)
                .lineToLinearHeading(new Pose2d(58,23, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(63,23, Math.toRadians(180)))
                .addTemporalMarker(()->{lift.moveLift(0.1f);})
                .addTemporalMarker(()->{lift.moveLift(-0.01f);})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.pivotEnding(); outtake.releaseMain();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{ outtake.releasePixels();})
                .waitSeconds(0.75)
                .addTemporalMarker(()-> {outtake.pivotStart(); lift.moveLift(0.3);})
                .lineToLinearHeading(new Pose2d(57,15, Math.toRadians(180)))
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
        dropper.Hold();
        while (!opModeIsActive()) {

            VisionBlueFar.Location location = VisionBlueFar.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }
        outtake.lockSecondary();
        outtake.releaseMain();
        waitForStart();

        while (opModeIsActive()) {


            VisionBlueFar.Location location = VisionBlueFar.getLocation();


            switch (location) {
                case NOT_FOUND:
                    drive.followTrajectorySequence(leftMovementOne);
                    sleep(3000000);
                    stop();
                    break;
                case MIDDLE:
                    drive.followTrajectorySequence(middleMovementOne);
                    sleep(30000000);
                    break;

                case RIGHT:
                    drive.followTrajectorySequence(rightMovementOne);
                    sleep(300000000);
                    break;
            }


            stop();
        }

    }

}

