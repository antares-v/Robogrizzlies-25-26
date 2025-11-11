package org.firstinspires.ftc.teamcode.mech.intake;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
//TODO: Integrate CV with this
public class intake {
    DcMotor leftIntake, rightIntake;
    public static double intake_constant = 1.0;
    public double targetValue = 650; //Intensity Threshold
    public intake(LinearOpMode l){
        leftIntake = l.hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = l.hardwareMap.get(DcMotor.class, "rightIntake");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        //l.hardwareMap.get(ColorSensor.class, "color_sensor");
    }
    public void takeIn(){
        leftIntake.setPower(1.0);
        leftIntake.setPower(1.0);
    }
    public void eject(){
        leftIntake.setPower(-1.0);
        leftIntake.setPower(-1.0);
    }
    public void stop(){
        leftIntake.setPower(0);
        leftIntake.setPower(0);
    }
    public boolean busy(){
        return (leftIntake.getPower()<0.1 || rightIntake.getPower()<0.1);
    }
}
