package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Camera + Drive Test")
public class CameraDrive extends LinearOpMode {

    EVALibrary robot = new EVALibrary();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        int startAngle = robot.getAngle();


        robot.knockMinerals(telemetry);
        robot.drive(0.2, 0.2);
        robot.sleep(2150);
        robot.stopDrive();
        robot.knockMinerals(telemetry);
        robot.drive(0.2, 0.2);
        robot.sleep(2160);
        robot.stopDrive();
        int postAngle = robot.getAngle();
        if ((startAngle + postAngle) < -20) {
            robot.turn(postAngle + 30, 0.3, EVALibrary.Rotation.CLOCKWISE);
        } else if ((startAngle + postAngle) < -20) {
            robot.turn(postAngle + 30, 0.3, EVALibrary.Rotation.CLOCKWISE);
        }





    }
}


