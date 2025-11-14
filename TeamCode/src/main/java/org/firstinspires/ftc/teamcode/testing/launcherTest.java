package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import org.firstinspires.ftc.teamcode.mech.mainIntakeOuttake.launcher;

@TeleOp
public class launcherTest extends LinearOpMode{
    CRServo leftservo;
    CRServo rightservo;
    LinearOpMode lom;
    DcMotor launcher;
    @Override
    public void runOpMode() {
        leftservo = hardwareMap.get(CRServo.class, "leftservo");
        rightservo = hardwareMap.get(CRServo.class, "rightservo");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                leftservo.setDirection(DcMotorSimple.Direction.FORWARD);
                leftservo.setPower(1);
                rightservo.setDirection(DcMotorSimple.Direction.REVERSE);
                rightservo.setPower(1);
                launcher.setPower(1);
            }
            if (gamepad1.a) {
                leftservo.setDirection(DcMotorSimple.Direction.REVERSE);
                leftservo.setPower(1);
                rightservo.setDirection(DcMotorSimple.Direction.FORWARD);
                rightservo.setPower(1);
                launcher.setPower(1);
            }

 //           telemetry.addData("servoPos", servo.getPosition());
  //          telemetry.update();
        }
    }
}
