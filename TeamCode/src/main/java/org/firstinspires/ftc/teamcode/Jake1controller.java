package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Jake1controller", group = "TeleOp")
public class Jake1controller extends LinearOpMode {
    DcMotorEx FrontLeft,FrontRight,BackLeft,BackRight,Slider,Arm;
    Servo grabber;
    Servo wrist;
    //DistanceSensor distance;

    double driveSpeedFR = 0.7;
    double driveSpeedFL = 0.62;
    double driveSpeedBR = 0.7;
    double driveSpeedBL = 0.62;

    public void runOpMode() throws InterruptedException
    {

        // motor declarations
        FrontLeft = hardwareMap.get(DcMotorEx.class,"front left");
        FrontRight = hardwareMap.get(DcMotorEx.class, "front right");
        BackLeft = hardwareMap.get(DcMotorEx.class, "back left");
        BackRight = hardwareMap.get(DcMotorEx.class, "back right");
        //distance =  hardwareMap.get(DistanceSensor.class, "distance");
        Slider = hardwareMap.get(DcMotorEx.class, "slider");
        Arm = hardwareMap.get(DcMotorEx.class, "arm");

        // servo declarations
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber = hardwareMap.get(Servo.class,"grabber");

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Slider.setDirection(DcMotorSimple.Direction.REVERSE);

        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setTargetPosition(0);
        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.setPower(0.5);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.5);



        waitForStart();
        Slider.setTargetPosition(100);

        while(opModeIsActive()){

            // drive calculations
            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // only drives when input is there
            if(Math.abs(vert) > .1 || Math.abs(horz) > .1 || Math.abs(rotate) > .1){
                Drive(vert,horz,rotate);
            }
            else{
                Drive(0,0,0);
            }

            /*if(gamepad1.y){
                Slider.setTargetPosition(2000);
            }
            if(gamepad1.a){
                Slider.setTargetPosition(0);
            }
            if(gamepad1.b)
            {
                Slider.setTargetPosition(1000);
            }
            if(gamepad1.x)
            {
                Slider.setTargetPosition(2850);
            }

             */

            // grabber closed preset
            if(gamepad1.left_bumper){
                grabber.setPosition(.7);
            }

            // grabber open preset
            if(gamepad1.right_bumper){
                grabber.setPosition(.5);
            }

            if(gamepad1.x) {
                Arm.setTargetPosition(Arm.getCurrentPosition() - 30);
            }

            if(gamepad1.y) {
                Arm.setTargetPosition(Arm.getCurrentPosition() + 30);
            }

            if(gamepad1.a) {
                wrist.setPosition(wrist.getPosition() + .01);
            }

            if(gamepad1.b) {
                wrist.setPosition(wrist.getPosition() - .01);
            }

            // wrist preset for down position
            if(gamepad1.dpad_down) {
                wrist.setPosition(0.67);
            }


            // telemetry for testing
            telemetry.addData("slider position", Slider.getCurrentPosition());
            telemetry.addData("Grabber position: ", grabber.getPosition());
            telemetry.addData("Wrist position: ", wrist.getPosition());
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }

    // drive calculations
    public void Drive(double vert, double horz, double rotate){
        double frdrive = -vert - horz - rotate;
        double fldrive = -vert + horz + rotate;
        double brdrive = -vert + horz - rotate;
        double bldrive = -vert - horz + rotate;

        // finding maximum drive for division below
        double max = Math.abs(Math.max(Math.abs(frdrive),Math.max(Math.abs(fldrive),Math.max(Math.abs(brdrive),Math.abs(bldrive)))));

        // power calculations
        FrontRight.setPower(driveSpeedFR * frdrive / max);
        FrontLeft.setPower(driveSpeedFL * fldrive / max);
        BackRight.setPower(driveSpeedBR * brdrive / max);
        BackLeft.setPower(driveSpeedBL * bldrive / max);









    }
}
