package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

import kotlin.math.UMathKt;
import sun.text.ComposedCharIter;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startingPose = new Pose2d(-40,-23, Math.toRadians(180));
//
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setStartPose(startingPose)
                .setDimensions(14.8125, 16.8125)
                .setConstraints(42.5, 80, 4, 4, 13.18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startingPose)
                                .lineToLinearHeading(new Pose2d(-30, 34, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-45.5,10,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(28, 9, Math.toRadians(180)))
                                // getting lift read
                                .splineToConstantHeading(new Vector2d(56, 40.75), Math.toRadians(50))
                                .lineToLinearHeading(new Pose2d(61,40, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(28, 8), Math.toRadians(0))




                                .build()
                                );
        // ,
        //                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
        //                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

