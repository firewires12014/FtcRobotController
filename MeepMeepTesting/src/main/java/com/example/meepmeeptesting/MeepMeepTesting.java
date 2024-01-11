package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startingPose = new Pose2d(-39.5,-63.6, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setStartPose(startingPose)
                .setDimensions(14.8125, 16.8125)
                .setConstraints(42.5, 80, 4, 4, 13.18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startingPose)
                                .setReversed(true)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-34,-43, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-32,-35, Math.toRadians(90)))
                                //put purple drop here
                                .lineToLinearHeading(new Pose2d(-35,-25, Math.toRadians(90)))
                                .splineTo(new Vector2d(32,-10),Math.toRadians(0))
                                .splineTo(new Vector2d(45,-43),Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(48,-43, Math.toRadians(0)))
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

