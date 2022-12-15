package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Jake2controller", group = "TeleOp")
public class Jake2controller extends LinearOpMode {
    DcMotorEx FrontLeft,FrontRight,BackLeft,BackRight,Slider;
    Servo grabber;
    //DistanceSensor distance;

    double driveSpeedFR = -0.7;
    double driveSpeedFL = -0.62;
    double driveSpeedBR = -0.7;
    double driveSpeedBL = -0.62;

    public void runOpMode() throws InterruptedException
    {
        FrontLeft = hardwareMap.get(DcMotorEx.class,"front left");
        FrontRight = hardwareMap.get(DcMotorEx.class, "front right");
        BackLeft = hardwareMap.get(DcMotorEx.class, "back left");
        BackRight = hardwareMap.get(DcMotorEx.class, "back right");
        //distance =  hardwareMap.get(DistanceSensor.class, "distance");
        Slider = hardwareMap.get(DcMotorEx.class, "slider");

        grabber = hardwareMap.get(Servo.class,"grabber");

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Slider.setDirection(DcMotorSimple.Direction.REVERSE);

        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setTargetPosition(0);
        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.setPower(1);



        waitForStart();

        while(opModeIsActive()){
            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if(Math.abs(vert) > .1 || Math.abs(horz) > .1 || Math.abs(rotate) > .1){
                Drive(vert,horz,rotate);
            }
            else{
                Drive(0,0,0);
            }

            if(gamepad2.y){
                Slider.setTargetPosition(2000);
            }
            if(gamepad2.a){
                Slider.setTargetPosition(0);
            }
            if(gamepad2.b)
            {
                Slider.setTargetPosition(1300);
            }
            if(gamepad2.x)
            {
                Slider.setTargetPosition(2850);
            }
            if(gamepad2.left_bumper){
                grabber.setPosition(.35);
            }
            if(gamepad2.right_bumper){
                grabber.setPosition(.53);
            }
            if(gamepad1.dpad_right)
            {

            }




            telemetry.addData("slider position", Slider.getCurrentPosition());
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }

    public void Drive(double vert, double horz, double rotate){
        double frdrive = vert + horz + rotate;
        double fldrive = vert - horz - rotate;
        double brdrive = vert - horz + rotate;
        double bldrive = vert + horz - rotate;

        double max = Math.abs(Math.max(Math.abs(frdrive),Math.max(Math.abs(fldrive),Math.max(Math.abs(brdrive),Math.abs(bldrive)))));

        FrontRight.setPower(driveSpeedFR * frdrive / max);
        FrontLeft.setPower(driveSpeedFL * fldrive / max);
        BackRight.setPower(driveSpeedBR * brdrive / max);
        BackLeft.setPower(driveSpeedBL * bldrive / max);









    }
}
