package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="EVA EVATeleOp")
public class EVATeleOp extends LinearOpMode {

    EVALibrary robot = new EVALibrary();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            robot.drive(-gamepad1.left_stick_x, -gamepad1.right_stick_x);


            if (gamepad1.left_bumper && robot.isLiftButtonUnpressed()) {
                robot.landerMotor(0.3);

            } else if (gamepad1.left_trigger > 0) {

                robot.landerMotor(-0.3);

            } else { robot.landerMotor(0); }



            if (gamepad1.right_bumper) {
                robot.mineralSpinner(0.3);

            } else if (gamepad1.right_trigger > 0) {

                robot.mineralSpinner(-0.3);

            } else { robot.mineralSpinner(0); }



            }


    }
}
