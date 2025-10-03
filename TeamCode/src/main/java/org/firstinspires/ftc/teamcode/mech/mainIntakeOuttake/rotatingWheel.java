package org.firstinspires.ftc.teamcode.mech.mainIntakeOuttake;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class rotatingWheel {
    Servo spinServo;
    double
    LinearOpMode l;
    public rotatingWheel(LinearOpMode l) {
        spinServo = new Servo(l.hardwareMap, "Spindexer", true);
    }
}
