package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;


@Autonomous(name="Lucy", group = "Robot")
@Disabled
public class Lucy extends LinearOpMode {

    DriveTrain driveTrain = new DriveTrain(hardwareMap);

    @Override
    public void runOpMode() {
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveTrain.turnToHeading(0.5,90);
            driveTrain.holdHeading(0.25,90,0.5);
            driveTrain.turnToHeading(0.5,180);
            driveTrain.holdHeading(0.25,180,0.5);
            driveTrain.turnToHeading(0.5,270);
            driveTrain.holdHeading(0.25,270,0.5);
            driveTrain.turnToHeading(0.5,360);
            driveTrain.holdHeading(0.25,360,0.5);
          sleep(2000000);
        }
    }
}
