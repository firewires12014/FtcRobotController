package org.firstinspires.ftc.teamcode.Auto;

import android.os.FileUriExposedException;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionBlueFar;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BlueSolo", group = "Robot")
public class BlueSolo extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionBlueFar VisionBlueFar = new VisionBlueFar(telemetry); // camera stuff
    Mecanum drive;
    Outtake outtake;
    Lift lift;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamBlue"), cameraMonitorViewId);
        camera.setPipeline(VisionBlueFar); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        drive = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        Pose2d startingPose = new Pose2d(-39.5,63.6, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        lift.lowerLift();
        outtake.lockPrimary();
        // Trajectory Declaration

        // right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-52, 32, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ intake.up(); lift.liftToHeight(60); outtake.autoDrop();})
                //drop purple pixel
                .addTemporalMarker(()->outtake.releaseMain())
                .waitSeconds(0.2)
                //getting into position for first white pixel
                .addTemporalMarker(()-> {lift.liftToHeight(50);lift.holdLift();})
                // go to stack
                        .lineToLinearHeading(new Pose2d(-47.25,10,Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> { intake.grabOneSpecial();intake.in();outtake.intakePosition();})
                .waitSeconds(0.75)
                // stops intake and lowers lift
                .addTemporalMarker(()->{intake.die();lift.lowerLift();})
                .waitSeconds(.2)
                // lock pixels
                .addTemporalMarker(()-> {outtake.lockPixels(); })
                .waitSeconds(0.15)
//                .addTemporalMarker(()-> {intake.outRoller();})
                //drive across field
                .lineToLinearHeading(new Pose2d(25, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // getting lift ready
                .UNSTABLE_addTemporalMarkerOffset(-1.25,()->{intake.outRoller(); })
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{lift.liftToHeight(300); lift.holdLift(); outtake.diffyPosition(3); })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(6); intake.die();})
                //driving to backboard
                //first drop
                .splineToConstantHeading(new Vector2d(48, 29), Math.toRadians(0),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(62,29,Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releaseMain();})
                .waitSeconds(0.05)
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{outtake.intakePosition(); lift.lowerLift();})
                                .splineToConstantHeading(new Vector2d(25, 11), Math.toRadians(180),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                //puts lift down
                .UNSTABLE_addTemporalMarkerOffset(-0.75 ,()->{outtake.diffyPosition(3); intake.grabTwo();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-0.15,()->{ lift.lowerLift();})
                //begins intaking
                .addTemporalMarker(()-> {intake.grabTwo(); intake.in();})
                //drives back to to stack
                .lineToLinearHeading(new Pose2d(-48.5, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{intake.grabThree();})
                //put lift in correct position
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->{ lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.25)
                //drive across field
                .UNSTABLE_addTemporalMarkerOffset(.5, ()->{intake.score();})
                .lineToLinearHeading(new Pose2d(25, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-0.85,()->{intake.killRoller(); lift.lowerLift();}) //kill roller
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()-> {outtake.lockPixels(); })
                .addTemporalMarker(()-> {intake.outRoller();})
                .UNSTABLE_addTemporalMarkerOffset(.75, ()-> {intake.die();})
//                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {lift.liftToHeight(250); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{outtake.diffyPosition(5);})
                //moves to spot to release pixels
                //second drop
                .splineToConstantHeading(new Vector2d(50, 35), Math.toRadians(0),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(61,38,Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.125)
                .splineToConstantHeading(new Vector2d(25, 10), Math.toRadians(180),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.intakePosition();})
                //puts lift down
                .UNSTABLE_addTemporalMarkerOffset(-0.6 ,()->{outtake.diffyPosition(3); intake.grabTwo();})
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{ lift.lowerLift();})
                .addTemporalMarker(()-> {intake.grabFour(); intake.in();})
                .waitSeconds(.1)
                //drives back to to stack
                .lineToLinearHeading(new Pose2d(-47.75, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()-> {intake.score();})
                .UNSTABLE_addTemporalMarkerOffset(-1.25,()->{ lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> {intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->{intake.score();})
                //drive across field
                .lineToLinearHeading(new Pose2d(25, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-0.85,()->{intake.killRoller(); lift.lowerLift();}) //kill roller
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.lockPixels(); })
                .addTemporalMarker(()-> {intake.outRoller();})
                .UNSTABLE_addTemporalMarkerOffset(.75, ()-> {intake.die();})
//                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {lift.liftToHeight(275); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{outtake.diffyPosition(5);})
                //moves to spot to release pixels
                .splineToConstantHeading(new Vector2d(48, 35), Math.toRadians(0),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(61,35 ,Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //Final Drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.125)
                .splineToConstantHeading(new Vector2d(55, 35), Math.toRadians(180),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.intakePosition();})
                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-44, 24, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ intake.up(); lift.liftToHeight(60); outtake.autoDrop();})
                //drop purple pixel
                .addTemporalMarker(()->outtake.releaseMain())
                .waitSeconds(0.2)
                //getting into position for first white pixel
                .addTemporalMarker(()-> {intake.in();lift.liftToHeight(50);lift.holdLift();})
                // go to stack
                .lineToLinearHeading(new Pose2d(-47,10,Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> { intake.grabOneSpecial();lift.liftToHeight(40);lift.holdLift();outtake.intakePosition();})
                .waitSeconds(0.6)
                // stops intake and lowers lift
                .addTemporalMarker(()->{intake.die();lift.lowerLift();})
                .waitSeconds(.3)
                // lock pixels
                .addTemporalMarker(()-> {outtake.lockPixels(); })
                .waitSeconds(0.15)
                .addTemporalMarker(()-> {intake.outRoller();})
                //drive across field
                .lineToLinearHeading(new Pose2d(25, 9, Math.toRadians(180)))
                // getting lift ready
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ intake.die();lift.liftToHeight(275); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(6);})
                //driving to backboard
                //first drop
                .splineToConstantHeading(new Vector2d(48.5, 35), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(61,36, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releaseMain();})
                .waitSeconds(0.09)
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{outtake.intakePosition(); lift.lowerLift();})
                .splineToConstantHeading(new Vector2d(25, 11), Math.toRadians(180),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                //puts lift down
//                .UNSTABLE_addTemporalMarkerOffset(-0.75 ,()->{outtake.diffyPosition(3); intake.grabTwo();})
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-0.15,()->{ lift.lowerLift();})
                //begins intaking
                .addTemporalMarker(()-> {intake.grabTwo(); intake.in();})
                //drives back to to stack
                .lineToLinearHeading(new Pose2d(-47, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{intake.grabThree();})
                //put lift in correct position
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->{ lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.1)
                //drive across field
                .UNSTABLE_addTemporalMarkerOffset(.3, ()->{intake.score();})
                .lineToLinearHeading(new Pose2d(25, 9, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->{intake.killRoller(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.lockPixels(); })
                .addTemporalMarker(()-> {intake.outRoller();})
                .UNSTABLE_addTemporalMarkerOffset(.75, ()-> {intake.die();})
//                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {lift.liftToHeight(250); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{outtake.diffyPosition(5);})
                //moves to spot to release pixels
                //second drop
                .splineToConstantHeading(new Vector2d(48.5, 30), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(61, 30, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(25, 10), Math.toRadians(180),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.intakePosition();})
                //puts lift down
                .UNSTABLE_addTemporalMarkerOffset(-0.6 ,()->{outtake.diffyPosition(3); intake.grabTwo();})
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{ lift.lowerLift();})
                .addTemporalMarker(()-> {intake.grabFour(); intake.in();})
                .waitSeconds(.15)
                //drives back to to stack
                .lineToLinearHeading(new Pose2d(-48, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()-> {intake.score();})
                .UNSTABLE_addTemporalMarkerOffset(-1.25,()->{ lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> {intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->{intake.score();})
                //drive across field
                .lineToLinearHeading(new Pose2d(25, 9, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->{intake.killRoller(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.lockPixels(); })
                .addTemporalMarker(()-> {intake.outRoller();})
                .UNSTABLE_addTemporalMarkerOffset(.75, ()-> {intake.die();})
//                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {lift.liftToHeight(250); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{outtake.diffyPosition(5);})
                //moves to spot to release pixels
                .splineToConstantHeading(new Vector2d(47.5, 31), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(61, 31, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //Final Drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(55, 28, Math.toRadians(180)))
                .addTemporalMarker(()->{outtake.intakePosition();})
                .build();
        //left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-30, 34, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ intake.up(); lift.liftToHeight(60); outtake.autoDrop();})
                //drop purple pixel
                .addTemporalMarker(()->outtake.releaseMain())
                .waitSeconds(0.2)
                //getting into position for first white pixel
                .addTemporalMarker(()-> {intake.in();lift.liftToHeight(50);lift.holdLift();})
                // go to stack
                .lineToLinearHeading(new Pose2d(-45.5,10,Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> { intake.grabOneSpecial();lift.liftToHeight(40);lift.holdLift();outtake.intakePosition();})
                .waitSeconds(0.6)
                // stops intake and lowers lift
                .addTemporalMarker(()->{intake.die();lift.lowerLift();})
                .waitSeconds(.3)
                // lock pixels
                .addTemporalMarker(()-> {outtake.lockPixels(); })
                .waitSeconds(0.15)
                .addTemporalMarker(()-> {intake.outRoller();})
                //drive across field
                .lineToLinearHeading(new Pose2d(28, 9, Math.toRadians(180)))
                // getting lift ready
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ intake.die();lift.liftToHeight(275); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(0);})
                //driving to backboard
                //first drop
                .splineToConstantHeading(new Vector2d(56, 40.75), Math.toRadians(50))
                .lineToLinearHeading(new Pose2d(61,40, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releaseMain();})
                .waitSeconds(0.09)
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{outtake.intakePosition(); lift.lowerLift();})
                .splineToConstantHeading(new Vector2d(29, 8), Math.toRadians(180),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                //puts lift down
//                .UNSTABLE_addTemporalMarkerOffset(-0.75 ,()->{outtake.diffyPosition(3); intake.grabTwo();})
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-0.15,()->{ lift.lowerLift();})
                //begins intaking
                .addTemporalMarker(()-> {intake.grabTwo(); intake.in();})
                //drives back to to stack
                .lineToLinearHeading(new Pose2d(-47, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{intake.grabThree();})
                //put lift in correct position
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->{ lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.1)
                //drive across field
                .UNSTABLE_addTemporalMarkerOffset(.3, ()->{intake.score();})
                .lineToLinearHeading(new Pose2d(28, 9, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->{intake.killRoller(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.lockPixels(); })
                .addTemporalMarker(()-> {intake.outRoller();})
                .UNSTABLE_addTemporalMarkerOffset(.75, ()-> {intake.die();})
//                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {lift.liftToHeight(250); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{outtake.diffyPosition(5);})
                //moves to spot to release pixels
                //second drop
                .splineToConstantHeading(new Vector2d(59, 32), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(61, 32, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(28, 8), Math.toRadians(180),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.intakePosition();})
                //puts lift down
                .UNSTABLE_addTemporalMarkerOffset(-0.6 ,()->{outtake.diffyPosition(3); intake.grabTwo();})
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{ lift.lowerLift();})
                .addTemporalMarker(()-> {intake.grabFour(); intake.in();})
                .waitSeconds(.15)
                //drives back to to stack
                .lineToLinearHeading(new Pose2d(-48.5, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()-> {intake.score();})
                .UNSTABLE_addTemporalMarkerOffset(-1.25,()->{ lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> {intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->{intake.score();})
                //drive across field
                .lineToLinearHeading(new Pose2d(28, 9, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->{intake.killRoller(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.lockPixels(); })
                .addTemporalMarker(()-> {intake.outRoller();})
                .UNSTABLE_addTemporalMarkerOffset(.75, ()-> {intake.die();})
//                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {lift.liftToHeight(250); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{outtake.diffyPosition(5);})
                //moves to spot to release pixels
                .splineToConstantHeading(new Vector2d(59, 32), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(61, 32, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //Final Drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(55, 28, Math.toRadians(180)))
                .addTemporalMarker(()->{outtake.intakePosition();})
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

            VisionBlueFar.Location location = VisionBlueFar.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {

            VisionBlueFar.Location location = VisionBlueFar.getLocation();
            telemetry.addData("Location: ", location);

            switch (location) {
                case NOT_FOUND:
                    intake.allUp();
                    drive.followTrajectorySequence(leftMovementOne);
                    sleep(3000000);
                    break;
                case MIDDLE:
                    intake.allUp();
                    drive.followTrajectorySequence(middleMovementOne);
                    sleep(300000);
                    break;
                case RIGHT:
                    intake.allUp();
                    drive.followTrajectorySequence(rightMovementOne);
                    sleep(3000000);
                    break;
            }


            sleep(30000);
        }

    }

}

