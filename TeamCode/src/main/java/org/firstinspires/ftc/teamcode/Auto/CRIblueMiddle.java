package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(name = "CRIblueMiddle", group = "Robot")
public class CRIblueMiddle extends LinearOpMode {
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
        outtake.lockPixels();
        Pose2d startingPose = new Pose2d(-12,63.6, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // LEFT
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(2, 42, Math.toRadians(240)))
                .addTemporalMarker(()->{intake.allUp(); intake.outRollerSlow();})
                .waitSeconds(0.6)
                .addTemporalMarker(()->{ intake.die();})
                .setReversed(true)
                //at backboard
                .splineTo(new Vector2d(63, 38), Math.toRadians(0))
                .waitSeconds(.2)
                .lineToLinearHeading(new Pose2d(73, 38, Math.toRadians(180)))
                //arm/claw
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(230);lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(1);})
                .waitSeconds(.2)
                //yellow drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(.25, ()-> {intake.die();})
                .setReversed(false)
                //away from backboard
                .splineToConstantHeading(new Vector2d(32, 60), Math.toRadians(180))
                //reset arm/claw
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(3); outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-.8, ()-> {lift.lowerLift();})
                //through trusses to stack
                .lineToLinearHeading(new Pose2d(-49, 58, Math.toRadians(180)))
                //at stack
                .splineToConstantHeading(new Vector2d(-86, 34), Math.toRadians(180))
                .addTemporalMarker(()-> {lift.liftToHeight(50); lift.holdLift(); intake.in();})
                .waitSeconds(.3)
                .addTemporalMarker(()-> {intake.grabOne();})
                .waitSeconds(.75)
                .addTemporalMarker(()-> {intake.grabTwo();})
                .waitSeconds(.5)
                .setTangent(Math.toRadians(-90))
                //away from stack
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-49, 57), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-.6, ()-> {intake.die(); })
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {lift.lowerLift();})
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {outtake.lockPixels();})
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> {intake.outRoller();})
                //through truss to backboard
                .lineToLinearHeading(new Pose2d(22, 57, Math.toRadians(180)))
                //at backboard
                .splineToConstantHeading(new Vector2d(64, 35), Math.toRadians(0))
                .waitSeconds(.2)
                .lineToLinearHeading(new Pose2d(73, 35, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-.3, ()-> {intake.die();})
                //arm/claw
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(320); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(1);})
                .waitSeconds(.3)
                //white cycle drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.5)
                //park
                .lineToLinearHeading(new Pose2d(65, 40, Math.toRadians(180)))
                .waitSeconds(.2)
                //reset arm/diffy
                .addTemporalMarker(()-> {outtake.diffyPosition(3); outtake.intakePosition();})
                .waitSeconds(.5)
                .addTemporalMarker(()-> {lift.lowerLift();})
                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple
                .lineToLinearHeading(new Pose2d(-6, 30, Math.toRadians(220)))
                .addTemporalMarker(()->{intake.allUp(); intake.outRollerSlow();})
                .waitSeconds(0.6)
                .addTemporalMarker(()->{ intake.die();})
                .setReversed(true)
                //at backboard
                .splineTo(new Vector2d(65, 35.5), Math.toRadians(0))
                .waitSeconds(.2)
                .lineToLinearHeading(new Pose2d(73.5, 35.5, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(200);lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(1);})
                .waitSeconds(.2)
                //first drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(.25, ()-> {intake.die();})
                .setReversed(false)
                //away from backboard
                .splineToConstantHeading(new Vector2d(32, 60), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(3); outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-.8, ()-> {lift.lowerLift();})
                //through trusses to stack
                .lineToLinearHeading(new Pose2d(-49, 60, Math.toRadians(180)))
                //at stack
                .splineToConstantHeading(new Vector2d(-86, 35), Math.toRadians(180))
                .addTemporalMarker(()-> {lift.liftToHeight(50); lift.holdLift(); intake.in();})
                .waitSeconds(.3)
                .addTemporalMarker(()-> {intake.grabOne();})
                .waitSeconds(.75)
                .addTemporalMarker(()-> {intake.grabTwo();})
                .waitSeconds(.5)
                .setTangent(Math.toRadians(-90))
                //away from stack
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-49, 58), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-.6, ()-> {intake.die(); outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {lift.lowerLift();})
                .UNSTABLE_addTemporalMarkerOffset(-.3, ()-> {outtake.lockPixels();})
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> {intake.outRoller();})
                //through truss to backboard
                .lineToLinearHeading(new Pose2d(22, 58, Math.toRadians(180)))
                //at backboard
                .splineToConstantHeading(new Vector2d(65, 35), Math.toRadians(0))
                .waitSeconds(.2)
                .lineToLinearHeading(new Pose2d(73, 35, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-.3, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(330); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(1);})
                .waitSeconds(.1)
                //white pixel drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.2)
                //park
                .lineToLinearHeading(new Pose2d(68, 35, Math.toRadians(180)))
                .waitSeconds(.2)
                //reset arm/diffy
                .addTemporalMarker(()-> {outtake.diffyPosition(3); outtake.intakePosition();})
                .waitSeconds(.5)
                .addTemporalMarker(()-> {lift.lowerLift();})
                .build();
        //Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .setTangent(-240)
                .splineToLinearHeading(new Pose2d(-14, 35, Math.toRadians(180)),Math.toRadians(180))
                .addTemporalMarker(()->{intake.allUp(); intake.outRollerSlow();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ intake.die();})
                //add wait for partner if needed
                .setReversed(true)
                //spline away from purple
                .splineTo(new Vector2d(30, 50), Math.toRadians(0))
                //at backboard
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(210);lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(5);})
                .splineToConstantHeading(new Vector2d(65, 28), Math.toRadians(0))
                .waitSeconds(.1)
                .lineToLinearHeading(new Pose2d(73.5, 28, Math.toRadians(180)))
                .waitSeconds(.1)
                //first drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(.25, ()-> {intake.die();})
                .setReversed(false)
                //away from backboard
                .splineToConstantHeading(new Vector2d(32, 60), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(3); outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-.8, ()-> {lift.lowerLift();})
                //through trusses to stack
                .lineToLinearHeading(new Pose2d(-49, 60, Math.toRadians(180)))
                //at stack
                .splineToConstantHeading(new Vector2d(-85.5, 34.5), Math.toRadians(180))
                .addTemporalMarker(()-> {lift.liftToHeight(50); lift.holdLift(); intake.in();})
                .waitSeconds(.3)
                .addTemporalMarker(()-> {intake.grabOne();})
                .waitSeconds(.75)
                .addTemporalMarker(()-> {intake.grabTwo();})
                .waitSeconds(.5)
                .setTangent(Math.toRadians(-90))
                //away from stack
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-49, 57), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-.6, ()-> {intake.die(); })
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {lift.lowerLift();})
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {outtake.lockPixels();})
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> {intake.outRoller();})
                //through truss to backboard
                .lineToLinearHeading(new Pose2d(22, 57, Math.toRadians(180)))
                //at backboard
                .splineToConstantHeading(new Vector2d(65, 34), Math.toRadians(0))
                .waitSeconds(.2)
                .lineToLinearHeading(new Pose2d(74, 34, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-.3, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(210); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(1);})
                .waitSeconds(.1)
                //white cycle drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.2)
                //park
                .lineToLinearHeading(new Pose2d(70, 34, Math.toRadians(180)))
                .waitSeconds(.2)
                //reset arm/diffy
                .addTemporalMarker(()-> {outtake.diffyPosition(3); outtake.intakePosition();})
                .waitSeconds(.5)
                .addTemporalMarker(()-> {lift.lowerLift();})
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
                case LEFT: //left
                    drive.followTrajectorySequence(leftMovementOne);
                    stop();
                    break;
            }
            sleep(30000);
        }

    }

}



