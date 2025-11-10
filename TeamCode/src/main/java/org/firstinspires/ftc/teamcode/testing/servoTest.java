package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
Notes for spindexer values:
Ball Input: 0,0.4,0.7
Ball Output: 0.1,0.5,0.9
*/

@TeleOp
public class servoTest extends LinearOpMode{
    Servo servo;
    double curPos;
    LinearOpMode l;
    @Override
    public void runOpMode() {
        curPos = 0.0;
        servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                curPos += 0.1;
                sleep(100);
            }
            if (gamepad1.a) {
                curPos -= 0.1;
                sleep(100);
            }
            servo.setPosition(curPos);
            telemetry.addData("servoPos", servo.getPosition());
            telemetry.update();
        }
    }
}
