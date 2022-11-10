package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
    CONTROLS:
    gamepad1:
        left stick:     Driving
        right stick:    Rotation
        a (x):          close grabber
        b:              open grabber

    gamapad2:
        right stick:    manual sliding
        a:              base
        b:              low
        x:              med
        y:              high?
 */

@TeleOp(name = "Andrew opmode", group = "TeleOp")
public class AndrewOpMode extends LinearOpMode {
    DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight, Slider;
    Servo grabber;

    double verticalControl, horizontalControl, rotateControl, verticalSlowControl, horizontalSlowControl, sliderControl;

    //state machine enums
    enum DRIVESTATE{
        joyDrive,
        dpadDrive
    }
    enum SLIDESTATE{
        autoSlide,
        manualSlide
    }

    DRIVESTATE driveState = DRIVESTATE.joyDrive;
    SLIDESTATE slideState = SLIDESTATE.autoSlide;

    double driveSpeed = 0.8;
    double slowerDriveSpeed = 0.4;
    double slideSpeed = 0.3;
    double autoSlideSpeed = 0.6;
    double sens = 0.2;

    public void runOpMode() throws InterruptedException {
        //initialize motors and servos
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotorEx.class, "RearLeft");
        BackRight = hardwareMap.get(DcMotorEx.class, "RearRight");

        Slider = hardwareMap.get(DcMotorEx.class, "Slider");

        grabber = hardwareMap.get(Servo.class, "Grabber");

        //reverse motors properly
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Slider.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            //driving
            {
                //get gamepad joystick and dpad variables for control
                verticalControl = -1 * clipJoyInput(gamepad1.left_stick_y);
                horizontalControl = clipJoyInput(gamepad1.left_stick_x);
                rotateControl = clipJoyInput(gamepad1.right_stick_x);

                verticalSlowControl = getGamepad1DpadY();
                horizontalSlowControl = getGamepad1DpadX();

                switch (driveState){
                    case joyDrive:
                        //move wheels
                        Drive(verticalControl, horizontalControl, rotateControl, driveSpeed);

                        //condition to change state
                        if((getGamepad1DpadX() != 0)|| (getGamepad1DpadY() != 0)){
                            driveState = DRIVESTATE.dpadDrive;
                        }
                        break;
                    case dpadDrive:
                        //move wheels slowly
                        Drive(verticalSlowControl,horizontalSlowControl,rotateControl,slowerDriveSpeed);

                        //condition to change state
                        if((Math.abs(verticalControl) >= sens) || (Math.abs(horizontalControl) >= sens)){
                            driveState = DRIVESTATE.joyDrive;
                        }
                        break;
                }
            }

            //sliding
            {
                sliderControl = -1 * clipJoyInput(gamepad2.right_stick_y);

                switch(slideState){
                    case autoSlide:
                        //check each button and set target position
                        if(gamepad2.a){
                            autoSlide(0);
                        }
                        if(gamepad2.b){
                            autoSlide(1000);
                        }
                        if(gamepad2.x){
                            autoSlide(2000);
                        }
                        if(gamepad2.y){
                            autoSlide(2500);
                        }
                        //case to change state
                        if(Math.abs(sliderControl) >= sens){
                            slideState = SLIDESTATE.manualSlide;
                        }
                        break;
                    case manualSlide:
                        //control slider manually
                        manualSlide(sliderControl * slideSpeed);

                        //case to change state (any button are pressed)
                        if(gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y){
                            slideState = SLIDESTATE.autoSlide;
                        }
                        break;
                }
            }

            //grabbing
            {
                if(gamepad1.a){
                    grabber.setPosition(.33);
                }
                if(gamepad1.b){
                    grabber.setPosition(.53);
                }
            }

            //telemetry
            telemetry.addData("gamepad1leftstickx", gamepad1.left_stick_x);
            telemetry.addData("horz control", horizontalControl);
            telemetry.addData("gamepad1leftsticky", gamepad1.left_stick_y);
            telemetry.addData("vert control", verticalControl);
            telemetry.addData("gamepad1rightstickx", gamepad1.right_stick_x);
            telemetry.addData("rotate control", rotateControl);
            telemetry.addData("DriveState", driveState);
            telemetry.addData("SlideState", slideState);

            telemetry.update();
        }
    }

    public void Drive(double vert, double horz, double rotate, double power) {
        double frdrive = vert - horz - rotate;
        double fldrive = vert + horz + rotate;
        double brdrive = vert + horz - rotate;
        double bldrive = vert - horz + rotate;

        double max = Math.abs(Math.max(Math.abs(frdrive), Math.max(Math.abs(fldrive), Math.max(Math.abs(brdrive), Math.abs(bldrive)))));

        FrontRight.setPower(power * frdrive / max);
        FrontLeft.setPower(power * fldrive / max);
        BackRight.setPower(power * brdrive / max);
        BackLeft.setPower(power * bldrive / max);
    }

    public void autoSlide(int position){
        Slider.setPower(slideSpeed);

        if(Slider.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        Slider.setTargetPosition(position);
    }

    public void manualSlide(double control){
        if(Slider.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        Slider.setPower(control);
    }



    //method to make clip joystick input if it is less than sensitivity constant
    public double clipJoyInput(double input){
        if(Math.abs(input) < sens){
            return 0;
        }

        return Range.clip(input, -1, 1);
    }

    //methods to convert dpad from individual buttons into something similar to joystick inputs
    public double getGamepad1DpadX(){
        if(gamepad1.dpad_right){
            return 1.0;
        }
        if(gamepad1.dpad_left){
            return -1.0;
        }

        return 0.0;
    }

    public double getGamepad1DpadY(){
        if(gamepad1.dpad_up){
            return 1.0;
        }
        if(gamepad1.dpad_down){
            return -1.0;
        }

        return 0.0;
    }
}
