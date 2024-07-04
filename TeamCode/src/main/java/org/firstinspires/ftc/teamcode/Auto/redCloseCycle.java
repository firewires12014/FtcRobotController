package org.firstinspires.ftc.teamcode.Auto;

import android.os.FileUriExposedException;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionRedClose;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "redCloseCycle", group = "Robot")

public class redCloseCycle extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionRedClose VisionRedClose = new VisionRedClose(telemetry); // camera stuff
    Mecanum drive;
    Outtake outtake;
    Lift lift;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamRed"), cameraMonitorViewId);
        camera.setPipeline(VisionRedClose); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        drive = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        lift.lowerLift();
        outtake.lockPixels();
        Pose2d startingPose = new Pose2d(16,-63.6, Math.toRadians(180));

        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(31, -32, Math.toRadians(180)),
                       Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                       Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> {lift.liftToHeight(160); lift.holdLift();outtake.diffyPosition(3);intake.outSlow();})
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {intake.die();})
                .lineToLinearHeading(new Pose2d(54, -44, Math.toRadians(180)),      Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-2, ()-> {lift.liftToHeight(130); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1.5, ()-> {lift.holdLift(); outtake.diffyPosition(5);})
                .addTemporalMarker(()->{outtake.releasePixels();}) //first drop
                .waitSeconds(0.15)
                .splineToConstantHeading(new Vector2d(16,-58.5), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()-> {lift.lowerLift(); })
                .splineToConstantHeading(new Vector2d(-55,-58), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-64.75,-33.5, Math.toRadians(180)),  //drive to stack/ at stack
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.55, ()-> {intake.in(); intake.grabOne();lift.liftToHeight(50); lift.holdLift();}) //grab one
                .waitSeconds(0.15)
                .addTemporalMarker(()->{intake.grabTwoSpecial();})
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-60,-50, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-45,-59.2), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()-> {lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(45,-58, Math.toRadians(180))) //drive back
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(52.75,-39, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.2, ()-> {lift.liftToHeight(310); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.grabOne();outtake.diffyPosition(1);})
                .addTemporalMarker(()->{ outtake.releasePixels();}) //second drop
                .waitSeconds(0.15)
                .splineToConstantHeading(new Vector2d(16,-58.5), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift(); intake.up();})
                .splineToConstantHeading(new Vector2d(-55, -58), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-64.5,-32, Math.toRadians(180)),  //drive to stack/ at stack
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {intake.in(); intake.grabFour();lift.liftToHeight(50); lift.holdLift();}) //grab one
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-60,-50, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-45,-58), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(40,-58, Math.toRadians(180))) //drive back
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                .addTemporalMarker(()-> {intake.outRoller();})
                .lineToLinearHeading(new Pose2d(52,-37.5, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(310); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(5);})
                .addTemporalMarker(()->{ outtake.releasePixels();}) //second drop
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(50,-39, Math.toRadians(180)))
                .build();



        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(15, -22, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> {lift.liftToHeight(160); lift.holdLift();outtake.diffyPosition(3);intake.outSlow();})
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {intake.die();})
                .lineToLinearHeading(new Pose2d(50.75, -35.75, Math.toRadians(180)), //at back board
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-2, ()-> {lift.liftToHeight(100); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1.5, ()-> {lift.holdLift(); outtake.diffyPosition(5);})
                .addTemporalMarker(()->{outtake.releasePixels();}) //first drop
                .waitSeconds(0.15)
                .splineToConstantHeading(new Vector2d(16,-58.5), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()-> {lift.lowerLift(); })
                .splineToConstantHeading(new Vector2d(-55,-58), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-64.5,-34, Math.toRadians(180)),  //drive to stack/ at stack
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.3, ()-> {intake.in(); intake.grabOne();lift.liftToHeight(50); lift.holdLift();}) //grab one
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.grabTwo();})
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-60,-50, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-45,-59.2), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()-> {lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(45,-58, Math.toRadians(180))) //drive back
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(52.75,-38.75, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(310); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.grabOne();outtake.diffyPosition(1);})
                //second drop
                .addTemporalMarker(()->{ outtake.releasePixels();})
                .waitSeconds(0.075)
                .splineToConstantHeading(new Vector2d(16,-58.5), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(3); intake.up(); intake.out();})
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift();  intake.die();intake.up();})
                .splineToConstantHeading(new Vector2d(-55, -58), Math.toRadians(180))
                //drive to stack/ at stack
                .lineToLinearHeading(new Pose2d(-64.5,-33, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //grab one
                .UNSTABLE_addTemporalMarkerOffset(-.3, ()-> {intake.in(); intake.grabFour();lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(-60,-50, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-45,-58), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift();})
                //drive back
                .lineToLinearHeading(new Pose2d(40,-58, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                .addTemporalMarker(()-> {intake.outRoller();})
                //at backboard
                .lineToLinearHeading(new Pose2d(51,-36.5, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(310); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(5);})
                //third drop
                .addTemporalMarker(()->{ outtake.releasePixels();})
         .build();
        //Left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                 .lineToLinearHeading(new Pose2d(6, -32, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.1, ()-> {lift.liftToHeight(210); lift.holdLift();outtake.diffyPosition(3);intake.outSlow();})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> {intake.die();})
                //at back board
                .lineToLinearHeading(new Pose2d(50, -29, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(1); intake.allUp();})
                .addTemporalMarker(()->{outtake.releasePixels();intake.up();})
                .waitSeconds(.15)
                //to stack
                .splineToConstantHeading(new Vector2d(16,-58.5), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()-> {lift.lowerLift(); })
                .splineToConstantHeading(new Vector2d(-55,-58), Math.toRadians(180))
                //drive to stack/ at stack
                .lineToLinearHeading(new Pose2d(-63.6,-33.5, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //grab one
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {intake.in(); intake.grabOne();lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.grabTwoSpecial();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(-60,-50, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-45,-59.2), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()-> {lift.lowerLift();})
                //drive back
                .lineToLinearHeading(new Pose2d(45,-58, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(52,-41, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(310); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-.9, ()-> {intake.grabOne();outtake.diffyPosition(1);})
                //second drop
                .addTemporalMarker(()->{ outtake.releasePixels();})
                .waitSeconds(0.15)
                .splineToConstantHeading(new Vector2d(16,-58.5), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift(); intake.up();})
                .splineToConstantHeading(new Vector2d(-55, -58), Math.toRadians(180))
                //drive to stack/ at stack
                .lineToLinearHeading(new Pose2d(-63,-32.75, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //grab one
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {intake.in(); intake.grabFour();lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(-60,-50, Math.toRadians(180)), //temp change
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45,-58), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift();})
                //drive back
                .lineToLinearHeading(new Pose2d(40,-58, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                .addTemporalMarker(()-> {intake.outRoller();})
                .lineToLinearHeading(new Pose2d(52,-41, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(310); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(5);})
                //second drop
                .addTemporalMarker(()->{ outtake.releasePixels();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(50,-39, Math.toRadians(180)))
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
        waitForStart();

        while (opModeIsActive()) {




            VisionRedClose.Location location = VisionRedClose.getLocation();

            switch (location) {
                case LEFT:
                    intake.allUp();
                    drive.followTrajectorySequence(leftMovementOne);
                    sleep(300000);
                    break;

                case MIDDLE:
                   intake.allUp();
                    drive.followTrajectorySequence(middleMovementOne);
                    sleep(30000);
                    break;

                case RIGHT:
                  intake.allUp();
                    drive.followTrajectorySequence(rightMovementOne);
                    sleep(300000);
                    break;
            }
//
//
            sleep(30000);
        }

    }

}


