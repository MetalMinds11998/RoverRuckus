package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Camera Test")
public class CameraTest extends LinearOpMode {

    EVALibrary robot = new EVALibrary();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            int angle = robot.getAngle();
            telemetry.addData("z", angle);
            telemetry.update();
        }
    }
}
