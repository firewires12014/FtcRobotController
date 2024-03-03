package org.firstinspires.ftc.teamcode.Auto;

import android.os.FileUriExposedException;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionRedFar;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "redFarCycle", group = "Robot")
public class redFarCycle extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionRedFar VisionRedFar = new VisionRedFar(telemetry); // camera stuff
    Mecanum drive;
    Outtake outtake;
    Lift lift;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamRed"), cameraMonitorViewId);
        camera.setPipeline(VisionRedFar); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        drive = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        Pose2d startingPose = new Pose2d(-39.5,-63.6, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        lift.lowerLift();
        outtake.lockPrimary();
        // Trajectory Declaration

        // right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-45, -35, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-39, -35, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{lift.liftToHeight(60); outtake.autoDrop();})
                .addTemporalMarker(()->outtake.releaseMain())
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(-50, -25, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-50, -11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //pixel grab
                .lineToLinearHeading(new Pose2d(30, -11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{lift.liftToHeight(190); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(1);})
                .lineToLinearHeading(new Pose2d(52, -39, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(48, -40, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.diffyPosition(3);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{outtake.intakePosition();})
                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-59, -23, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{lift.liftToHeight(60); outtake.autoDrop();})
                .addTemporalMarker(()->{outtake.releaseMain();})
                .waitSeconds(0.55)
                .lineToLinearHeading(new Pose2d(-73,-12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.intakePosition();intake.inSlow();})
                .waitSeconds(.25)
                .addTemporalMarker(()-> {intake.in();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-64, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{intake.score();})
                .lineToLinearHeading(new Pose2d(-70, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.in();})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{intake.up();intake.die();lift.lowerLift();outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(180)), //drive over
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{lift.liftToHeight(190); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(1);})
                .lineToLinearHeading(new Pose2d(52, -34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.diffyPosition(3);})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.intakePosition(); lift.lowerLift();intake.superStack();})
                .lineToLinearHeading(new Pose2d(-71, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{outtake.intakePosition(); lift.lowerLift();intake.superStack(); intake.inSlow();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{intake.score();})
                .lineToLinearHeading(new Pose2d(-67, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.4)
                .addTemporalMarker(()->{intake.die(); lift.lowerLift(); outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(40, -13, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.lockPixels();lift.liftToHeight(300); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(3);})
                .lineToLinearHeading(new Pose2d(50,-30, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .build();
        //left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-65, -30, Math.toRadians(180)), //drive to purple drop
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(55); lift.holdLift();outtake.autoDrop();})
                .addTemporalMarker(()-> { outtake.releaseMain();}) // drop purple
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-65, -12, Math.toRadians(180)), //driving to stack
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // grab top pixel
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.intakePosition(); lift.liftToHeight(5); lift.holdLift();outtake.releasePixels();})
                // drive forward
                .lineToLinearHeading(new Pose2d(-73, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // start intaking
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.inSlow();})
                .waitSeconds(0.3) //put back to 0.4 if no work
                // backup
                .lineToLinearHeading(new Pose2d(-64, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // lift intake
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.25)
                // lower intake
                .addTemporalMarker(()->{intake.score();})
                // drive forward
                .lineToLinearHeading(new Pose2d(-70, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // intakes fast
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.in();})
                .waitSeconds(0.4)
                // stops intake and lowers lift
                .addTemporalMarker(()->{intake.die();lift.lowerLift();})
                .waitSeconds(.25)
                // grab 1st white pixel
                .addTemporalMarker(()-> {outtake.lockPixels(); })
                .waitSeconds(0.25)
                // drive across field
                .lineToLinearHeading(new Pose2d(30, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                //lock pixels
                .addTemporalMarker(()->{outtake.lockPixels();})
                //putting lift to release height
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.lockPixels();lift.liftToHeight(250); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(6);})
                //driving to spot to release pixels
                .lineToLinearHeading(new Pose2d(50,-28, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //release pixels
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.25)
                //getting ready to drive across field
                .lineToLinearHeading(new Pose2d(30, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //puts lift down
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(3); intake.superStack();})
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{outtake.intakePosition(); lift.lowerLift();})
                //drives to stack
                .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-71, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-72, -12, Math.toRadians(180)))
                //put lift in correct position and begins intaking
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ lift.liftToHeight(50);})
                .waitSeconds(0.40)
                .addTemporalMarker(()->{intake.in();})
                //backs up
                .lineToLinearHeading(new Pose2d(-62, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{intake.score();})
                //moves forward
                .lineToLinearHeading(new Pose2d(-67, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //intake fast
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.in();})
                .waitSeconds(0.4)
                //drive across field
                .lineToLinearHeading(new Pose2d(40, -13, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{intake.die(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-2, ()-> {outtake.lockPixels(); })
                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(300); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(6);})
                //moves to spot to release pixels
                .lineToLinearHeading(new Pose2d(50,-34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //release pixels
                .addTemporalMarker(()->{outtake.releasePixels();})
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

            VisionRedFar.Location location = VisionRedFar.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {

            VisionRedFar.Location location = VisionRedFar.getLocation();
            telemetry.addData("Location: ", location);

            switch (location) {
                case LEFT:
                    intake.score();
                    sleep(300);
                    intake.specialStack();
                    drive.followTrajectorySequence(leftMovementOne);
                    sleep(3000000);
                    break;
                case MIDDLE:
                    intake.specialStack();
                    drive.followTrajectorySequence(middleMovementOne);
                    sleep(300000);
                    break;
                case RIGHT:
                    intake.specialStack();
                    drive.followTrajectorySequence(rightMovementOne);
                    sleep(3000000);
                    break;
            }


            sleep(30000);
        }

    }

}

