package org.firstinspires.ftc.teamcode.mech.mainIntakeOuttake;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class rotatingWheel {
    Servo spinServo;
    priavte final double pos1 = 0.0;
    priavte final double pos2 = 0.0;
    priavte final double pos3 = 0.0;
    priavte final double pos4 = 0.0;
    priavte final double pos5 = 0.0;
    priavte final double pos6 = 0.0;
    private int posloc = 2;
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
    public void servopos4() {
        spinServo.setPosition(pos4);
        posloc = 4;
    }
    public void servopos5() {
        spinServo.setPosition(pos5);
        posloc = 5;
    }
    public void servopos6() {
        spinServo.setPosition(pos6);
        posloc = 6;
    }
    public int location(){
        return posloc;
    }

}
