package org.firstinspires.ftc.teamcode.mech.movement;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
public class wheel {
    //Define our motor for the wheel
    private DcMotorEx motor;
    //Constructor For Wheel:
    private Servo servo;
    public wheel(HardwareMap map, String s, boolean reverse){
        motor = map.get(DcMotorEx.class, s);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        if(reverse){
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }else{
            motor.setDirection(DcMotorEx.Direction.FORWARD);
        }
    }
    /*public servoV2(HardwareMap map, String s){
        servo = map.get(DcMotor.class, s);
    }*/
    public void setPower(double power){
        //if above 1
        motor.setVelocity(power);
    }
    public double getPower(){
        return motor.getVelocity();
    }
}
