package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class RobotMap {

    public Servo servoGheara = null;
    public Servo servoGhidaj = null;
    public DcMotor motor4Bar = null;
    public DcMotorEx extensieOuttake = null;
    public RobotMap(HardwareMap Init)
    {
        servoGheara = Init.get(Servo.class, "servoGheara");
        servoGhidaj = Init.get(Servo.class,"servoGhidaj");


        motor4Bar = Init.get(DcMotor.class, "motor4Bar");

        motor4Bar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4Bar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4Bar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensieOuttake = Init.get(DcMotorEx.class, "extensieOuttake");
        extensieOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensieOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensieOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensieOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}