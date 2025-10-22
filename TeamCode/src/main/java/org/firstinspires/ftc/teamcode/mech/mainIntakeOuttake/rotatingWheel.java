package org.firstinspires.ftc.teamcode.mech.mainIntakeOuttake;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class rotatingWheel {
    Servo spinServo;
    double pos1 = 0.0;
    double pos2 = 0.0;
    double pos3 = 0.0;
    int posloc = 2;
    LinearOpMode l;
    public rotatingWheel(LinearOpMode l) {
        spinServo = l.hardwareMap.get(Servo.class, "spinDexer");
    }

    public void servopos1() {
        spinServo.setPosition(pos1);
        posloc = 1;
    }
    public void servopos2() {
        spinServo.setPosition(pos2);
        posloc = 2;
    }
    public void servopos3() {
        spinServo.setPosition(pos3);
        posloc = 3;
    }
    public int location(){
        return posloc;
    }

}
