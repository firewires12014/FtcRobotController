package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;


@Autonomous(name="Lucy", group = "Robot")
public class Lucy extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot       = new RobotHardware(this);
    DriveTrain dt = new DriveTrain(this);

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            dt.turnToHeading(0.5,90);
            dt.holdHeading(0.25,90,0.5);
            dt.turnToHeading(0.5,180);
            dt.holdHeading(0.25,180,0.5);
            dt.turnToHeading(0.5,270);
            dt.holdHeading(0.25,270,0.5);
            dt.turnToHeading(0.5,360);
            dt.holdHeading(0.25,360,0.5);
          sleep(2000000);
        }
    }
}
