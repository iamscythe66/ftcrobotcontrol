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



    // list of methods

    // arm to drop position
    public void armBack() {
        Arm.setPower(armPower);
        Arm.setTargetPosition(-700);
        Arm.setPower(0.45);
        Arm.setTargetPosition(-1500);
    }

    // arm to pick up position
    public void armFront() {
        Arm.setPower(armPower);
        Arm.setTargetPosition(0);
    }

    // wrist to pick up position
    public void wristDown() {

        wrist.setPosition(0.76);
    }

    // wrist to drop position
    public void wristUp() {
        wrist.setPosition(0.13);
    }

    // list of power values
    public double slidePower = 0.5;

    public double armPower = 0.8;

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
        Slider.setPower(slidePower);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(armPower);



        waitForStart();

        while(opModeIsActive()){

            // drive calculations
            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            if (gamepad1.left_trigger > 0.2) {
                driveSpeed = 0.15
            } else {
                driveSpeed = 0.7
            }


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
                grabber.setPosition(0.51);
            }

            // grabber open preset
            if(gamepad1.right_bumper){
                grabber.setPosition(0.65);
            }

            if(gamepad1.x) {
                Arm.setTargetPosition(Arm.getCurrentPosition() - 100);
            }

            if(gamepad1.y) {
                Arm.setTargetPosition(Arm.getCurrentPosition() + 100);
            }

            // arm forward position
            if(gamepad1.a) {
                armFront();
                wristDown();
            }

            // arm back position
            if(gamepad1.b) {
                armBack();
                wristUp();
            }

            // arm to pick up position
            if(gamepad1.dpad_down) {
                armFront();
            }

            // arm to drop position
            if(gamepad1.dpad_up) {
                armBack();
            }

            // arm and wrist at the same time
            if(gamepad1.dpad_left) {
                armBack();
            }

            if(gamepad1.dpad_right) {
                armFront();
            }

            // telemetry for testing
            telemetry.addData("slider position", Slider.getCurrentPosition());
            telemetry.addData("Grabber position: ", grabber.getPosition());
            telemetry.addData("Wrist position: ", wrist.getPosition());
            telemetry.addData("Arm position: ", + Arm.getCurrentPosition());
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }

    // drive calculations

    double driveSpeed = 0.7;
    double driveTuningFR = 1.0;
    double driveTuningFL = 0.75;
    double driveTuningBR = 1.0;
    double driveTuningBL = 0.75;

    public void Drive(double vert, double horz, double rotate){
        double frdrive = -vert - horz - rotate;
        double fldrive = -vert + horz + rotate;
        double brdrive = -vert + horz - rotate;
        double bldrive = -vert - horz + rotate;

        // finding maximum drive for division below
        double max = Math.abs(Math.max(Math.abs(frdrive),Math.max(Math.abs(fldrive),Math.max(Math.abs(brdrive),Math.abs(bldrive)))));

        // power calculations
        FrontRight.setPower(driveSpeed * driveTuningFR * frdrive / max);
        FrontLeft.setPower(driveSpeed * driveTuningFL * fldrive / max);
        BackRight.setPower(driveSpeed * driveTuningBR * brdrive / max);
        BackLeft.setPower(driveSpeed * driveTuningBL * bldrive / max);









    }
}
