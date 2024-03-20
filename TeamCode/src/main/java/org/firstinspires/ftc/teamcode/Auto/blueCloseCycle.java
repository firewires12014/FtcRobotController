package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionBlueClose;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "blueCloseCycle", group = "Robot")
@Disabled
public class blueCloseCycle extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionBlueClose VisionBlueClose = new VisionBlueClose(telemetry); // camera stuff
    Mecanum drive;
    Outtake outtake;
    Lift lift;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamBlue"), cameraMonitorViewId);
        camera.setPipeline(VisionBlueClose); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        drive = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        //intake.resetIntake();
        outtake.intakePosition();
        lift.lowerLift();
        sleep(300);
        outtake.lockPixels();
        Pose2d startingPose = new Pose2d(16,63.6, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // LEFT
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(41, 32, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.stackHeight();})
                .lineToLinearHeading(new Pose2d(59, 44, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()-> {lift.liftToHeight(160); lift.holdLift();})
                .addTemporalMarker(()-> {outtake.diffyPosition(3); })
                .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.intakePosition();lift.liftToHeight(0); intake.score();})
              //pixel on back board
                .lineToLinearHeading(new Pose2d(42, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.stackHeight();})
                .lineToLinearHeading((new Pose2d(-47, 10,Math.toRadians(180))),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-50, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()-> {intake.in();})
                .waitSeconds(.8)
                .addTemporalMarker(()->{intake.stackHeightTwo();})
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(-46, 10, Math.toRadians(180)))
                .addTemporalMarker(()-> {intake.score();})
                .addTemporalMarker(()->{lift.liftToHeight(5);lift.holdLift();})
                .waitSeconds(0.3)
                .addTemporalMarker(()-> {intake.die();})
                .addTemporalMarker(()-> {intake.inSlow();})
                .lineToLinearHeading(new Pose2d(42, 10, Math.toRadians(180)), // drive across field
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()-> {intake.die();})
                .addTemporalMarker(()-> {lift.lowerLift();})
                .waitSeconds(.3)
                .addTemporalMarker(()-> {outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(61, 36, Math.toRadians(180)), // to backboard
                        Mecanum.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.25)
                .addTemporalMarker(()-> {lift.liftToHeight(160); lift.holdLift();})
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {outtake.diffyPosition(3); })
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {outtake.diffyPosition(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{outtake.intakePosition();lift.liftToHeight(0); intake.score();})



                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(34, 24, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.stack();})
                .lineToLinearHeading(new Pose2d(60, 36, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{lift.liftToHeight(160); lift.holdLift();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.diffyPosition(3);})
                .waitSeconds(0.25)
                .addTemporalMarker(()-> {outtake.releasePixels();})
                .waitSeconds(0.5)
                .build();
        //Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(18, 33, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.stack();})
                .lineToLinearHeading(new Pose2d(62, 27, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{lift.liftToHeight(160); lift.holdLift();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.diffyPosition(3);})
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {outtake.diffyPosition(0);})
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {outtake.releasePixels();})
                .waitSeconds(0.5)
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

            VisionBlueClose.Location location = VisionBlueClose.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            intake.auto();

            VisionBlueClose.Location location = VisionBlueClose.getLocation();
            telemetry.addData("Location:", location);
            telemetry.update();
            switch (location) {
                case NOT_FOUND:
                    drive.followTrajectorySequence(rightMovementOne);
                    stop();

                    break;
                case MIDDLE:
                    telemetry.addData("Middle:", "Activated");
                    telemetry.update();
                     drive.followTrajectorySequence(middleMovementOne);
                     stop();
                    break;
                case LEFT:
                    drive.followTrajectorySequence(leftMovementOne);
                    stop();
                    sleep(30000);
                    break;
            }
            sleep(30000);
        }

    }

}


