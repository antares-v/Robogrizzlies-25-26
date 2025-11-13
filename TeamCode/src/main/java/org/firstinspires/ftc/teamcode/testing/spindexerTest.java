package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
Notes for spindexer values:
[0,0.19,0.38,0.59,0.79,0.99]
*/

@TeleOp
public class spindexerTest extends LinearOpMode{
    Servo servo;
    double curPos;
    LinearOpMode l;
    int i = 0;
    @Override
    public void runOpMode() {
        curPos = 0.0;
        servo = hardwareMap.get(Servo.class, "servo");
        double[] spindexerPos = {0,0.19,0.38,0.59,0.79,0.99};
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y && i < spindexerPos.length-1) {
                i++;
                sleep(250);
            }
            if (gamepad1.a && i > 0) {
                i--;
                sleep(250);
            }
            servo.setPosition(spindexerPos[i]);
            telemetry.addData("servoPos", servo.getPosition());
            telemetry.addData("curPos", spindexerPos[i]);
            telemetry.addData("i", i);
            telemetry.update();
        }
    }
}
