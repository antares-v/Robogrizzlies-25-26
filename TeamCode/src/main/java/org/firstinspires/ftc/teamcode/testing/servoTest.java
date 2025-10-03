package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class servoTest extends LinearOpMode{
    Servo servo;
    double curPos;
    LinearOpMode l;
    @Override
    public void runOpMode() {
        curPos = 0.0;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                curPos += 0.1;
            }
            if (gamepad1.a) {
                curPos -= 0.1;
            }
            servo.setPosition(curPos);
            telemetry.addData("servoPos", servo.getPosition());
            telemetry.update();
        }
    }
}
