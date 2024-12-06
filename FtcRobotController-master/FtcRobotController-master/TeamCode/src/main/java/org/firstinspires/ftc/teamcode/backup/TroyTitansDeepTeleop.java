/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot.
 * An external "Robot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="TroyTitansDeepTeleop", group = "Gobuilda")
public class TroyTitansDeepTeleop extends LinearOpMode
{
    final double SAFE_DRIVE_SPEED   = 0.5 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED  = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     = 0.5 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double HEADING_HOLD_TIME  = 10.0 ; // How long to hold heading once all driver input stops. (This Avoids effects of Gyro Drift)

    // local parameters
    ElapsedTime stopTime   = new ElapsedTime();  // Use for timeouts.
    boolean autoHeading    = false; // used to indicate when heading should be locked.


    private DcMotor slider;
    private Servo gripper;
    private Servo subIntake;
    private Servo subWrist;
    private TouchSensor touchslider;
    private Servo wrist;
    private DcMotor sliderSub;
    private DcMotor armLeft;
    private DcMotor armRight;
    int sliderhangposition;
    int subSliderClosedPosition;
    ElapsedTime runtime;
    ElapsedTime subRuntime;
    ElapsedTime sliderRunTime;
    ElapsedTime sliderSubRunTime;

    int sliderTimer;

    int SliderClosePosition;
    int SliderIntakePosition;
    int SliderSubIntakePosition;
    double subIntakeOff;
    int SliderExtendPosition;
    double wristSpecimenPick;
    double subWristNeutral;
    double wristUpPosition;

    double wristDeadBand;
    double wristDropPosition;
    double wristDownPosition;
    int gripperClosedPosition;
    int gripperOpenPosition;
    double subIntakeIn;
    int subIntakeOut;
    double subWristUp;
    double subWristDown;
    boolean autoSlider;
    boolean autoSubSlider;
    double running;

    private void init_variables() {
        // Slider Position
        sliderhangposition = 1400;
        // Vertical Slider Position
        SliderClosePosition = 0;
        SliderIntakePosition = 425;
        SliderExtendPosition = 3600;

        SliderSubIntakePosition = 1000;
        // Wrist on Vertical Slider Position
        wristSpecimenPick = 0.5;
        wristUpPosition = 0.85; //Neeed to add a button
        wristDownPosition = 0;  //Neeed to add a button
        wristDropPosition = 0.7;
        wristDeadBand = 0.03;
        // Gripper on Vertical Slider Position
        gripperClosedPosition = 1;
        gripperOpenPosition = 0;
        // Horizontal Submersible Slider Position
        subSliderClosedPosition = 0;
        // Intake on Submersible Slider Position
        subIntakeOff = 0.5;
        subIntakeIn = -1;
        subIntakeOut = 1;
        // Wrist on Submersible Slider Position
        subWristNeutral = 0.6;
        subWristDown = 1;
        subWristUp = 0;
        // Initiation
        autoSlider = false;
        autoSubSlider = false;
        running = 0;
        runtime = new ElapsedTime();
        subRuntime = new ElapsedTime();
        sliderRunTime = new ElapsedTime();
        sliderSubRunTime = new ElapsedTime();
        sliderTimer = 2000;

    }
    // get an instance of the "Robot" class.
    SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override public void runOpMode()
    {
        // Initialize the drive hardware & Turn on telemetry
        robot.initialize(true,true);

        slider = hardwareMap.get(DcMotor.class, "slider");
        gripper = hardwareMap.get(Servo.class, "gripper");
        subIntake = hardwareMap.get(Servo.class, "subIntake");
        subWrist = hardwareMap.get(Servo.class, "subWrist");
        wrist = hardwareMap.get(Servo.class, "wrist");
        sliderSub = hardwareMap.get(DcMotor.class, "sliderSub");
        armRight = hardwareMap.get(DcMotor.class, "axial");
        armLeft = hardwareMap.get(DcMotor.class, "lateral");
        //    touchslider = hardwareMap.get(TouchSensor.class, "touchslider");


        init_variables();
        Slider_init();
        arm_init();
        SliderSub_init();
        subIntake.setPosition(subIntakeOff);
        subWrist.setPosition(subWristUp);
        wrist.setPosition(wristUpPosition);
        gripper.setPosition(gripperClosedPosition);

        telemetry.addData("Status", "Initialized");
        waitForStart();

        while (opModeIsActive())
        {
            drive_process();
            arm_process();
            gamepadControls();
            runtime.reset();
            subRuntime.reset();
            /*if (touchslider.isPressed()) {
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // telemetry.addData("Sensor pressed", slider.getCurrentPosition());
            }*/
            //Turn off slider only if its not moving automatically.
            //Automatic movements stops at target position already.
            if(!autoSlider) {
                slider.setPower(0);
            }
            else if(autoSlider && (sliderRunTime.milliseconds() >= sliderTimer)) {
                slider.setPower(0);
                sliderRunTime.reset();
                autoSlider = false;
                sliderTimer =2000;
            }

            if(!autoSubSlider)
            {
                sliderSub.setPower(0);
            }
            else if (autoSubSlider && (sliderSubRunTime.milliseconds() >= 2000))
            {
                sliderSub.setPower(0);
                sliderSubRunTime.reset();
                autoSubSlider = false;
            }
            Manual_Mode_Slider();
            Manual_Mode_Slidersub();
            telemetry.addData("Slider Position", slider.getCurrentPosition());
            telemetry.addData("SubSlider Position", sliderSub.getCurrentPosition());

        }
    }

    private void drive_process() {
        robot.readSensors();

        // Allow the driver to reset the gyro by pressing both small gamepad buttons
        if(gamepad1.dpad_down){
            robot.resetHeading();
            robot.resetOdometry();
        }
        // read joystick values and scale according to limits set at top of this file
        double drive  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;      //  Fwd/back on left stick
        double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;     //  Left/Right on left stick
        double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;       //  Rotate on right stick
        boolean backbutton = false;

        //  OR... For special conditions, Use the DPAD to make pure orthoginal motions
        if (gamepad1.dpad_left) {
            strafe = -SAFE_DRIVE_SPEED / 2.0;
        } else if (gamepad1.dpad_right) {
            strafe = SAFE_DRIVE_SPEED / 2.0;
        }else if (gamepad1.back) {
            backbutton = true;
        }
        else if (gamepad1.left_bumper) {
            drive = drive*1.6;

        } else if (gamepad1.right_bumper) {
            drive = drive/2;
            strafe = strafe/2;
        }


        // This is where we heep the robot heading locked so it doesn't turn while driving or strafing in a straight line.
        // Is the driver turning the robot, or should it hold its heading?
        if (Math.abs(yaw) > 0.05) {
            // driver is commanding robot to turn, so turn off auto heading.
            autoHeading = false;
        } else if (!backbutton){
            // If we are not already locked, wait for robot to stop rotating (<2 deg per second) and then lock-in the current heading.
            if (!autoHeading && Math.abs(robot.getTurnRate()) < 2.0) {
                robot.yawController.reset(robot.getHeading());  // Lock in the current heading
                autoHeading = true;
            }
        }

        // If auto heading is on, override manual yaw with the value generated by the heading controller.
        if (autoHeading) {
            yaw = robot.yawController.getOutput(robot.getHeading());
        }

        if(backbutton) {
            backbutton = false;
            robot.turnRobot(180, 0.5);
            //robot.specimenMove(20,35,0.5);
        }
        else
        {
            //  Drive the wheels based on the desired axis motions
            robot.moveRobot(drive, strafe, yaw);
        }

        backbutton = false;

        // If the robot has just been sitting here for a while, make heading setpoint track any gyro drift to prevent rotating.
        if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
            if (stopTime.time() > HEADING_HOLD_TIME) {
                robot.yawController.reset(robot.getHeading());  // just keep tracking the current heading
            }
        } else {
            stopTime.reset();
        }
    }

    private void arm_process(){
        double armOpenPowwer = 0.8;
        double armClosePower = -0.8;
        if(gamepad2.left_trigger > 0)
        {
            armLeft.setPower(armOpenPowwer);
            armRight.setPower(armOpenPowwer);
        }
        else if (gamepad2.right_trigger > 0) {
            armLeft.setPower(armClosePower);
            armRight.setPower(armClosePower);
        }
        else
        {
            armLeft.setPower(0);
            armRight.setPower(0);
        }
    }

    private void arm_init() {
        armRight.setDirection(DcMotor.Direction.REVERSE);
        // armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armLeft.setDirection(DcMotor.Direction.FORWARD);
        //armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armLeft.setPower(0);
        armRight.setPower(0);
    }

    private void Slider_init() {
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotor.Direction.FORWARD);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setPower(0);
    }

    private void SliderSub_init() {

        sliderSub.setDirection(DcMotor.Direction.FORWARD);
        sliderSub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderSub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderSub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderSub.setPower(0);
    }

    private void subSlider_homePosition() {
        sliderSub.setTargetPosition(subSliderClosedPosition);
        sliderSub.setPower(1);
        sliderSub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        subIntake.setPosition(subIntakeOff);
    }

    private void subSlider_noFallPosition() {
        sliderSub.setTargetPosition(500);
        sliderSub.setPower(1);
        sliderSub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        subIntake.setPosition(subIntakeOff);
    }

    private void Manual_Mode_Slider() {

        runtime.reset();
        if (gamepad2.left_stick_y != 0 ) {
            //slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slider.setPower(-gamepad2.left_stick_y);
            autoSlider = false;
        }
        else if (!autoSlider)
        {
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setPower(0);
        /*    if (touchslider.isPressed()) {
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //telemetry.addData("Sensor pressed manual", slider.getCurrentPosition());
            }*/
        }

    }

    private void Manual_Mode_Slidersub() {

        subRuntime.reset();
        if (gamepad2.right_stick_y != 0) {
            sliderSub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sliderSub.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sliderSub.setPower(-gamepad2.right_stick_y/1.33);
            autoSubSlider = false;
        }
        else if(!autoSubSlider)
        {
            sliderSub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sliderSub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderSub.setPower(0);
        }

    }

    private void setSliderPosition(int sliderPosition) {
        sliderRunTime.reset();
        slider.setTargetPosition(sliderPosition);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autoSlider = true;
    }

    private void setSliderSubPosition(int sliderSubPosition) {
        sliderSubRunTime.reset();
        sliderSub.setTargetPosition(sliderSubPosition);
        sliderSub.setPower(1);
        sliderSub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autoSubSlider = true;
    }



    private void gamepadControls() {

        if (gamepad2.a) {
            //Home position
            sliderTimer = 2000;
            setSliderPosition(SliderClosePosition);
            wrist.setPosition(wristUpPosition);
            gripper.setPosition(gripperClosedPosition);
            // Retract Horizontal Submersible Slider
            setSliderSubPosition(subSliderClosedPosition);
            // Wrist on Submersible Up
            subWrist.setPosition(subWristUp);
            running = 1;
        } else if (gamepad2.b) {
            //Specimen Pick Positions
            // Horizontal Submersible Slider close to specimen Pick
            sliderTimer = 2000;
            subSlider_homePosition();
            // Vertical Slider Up and Away from Horozontal Slider
            setSliderPosition(SliderIntakePosition);
            wrist.setPosition(wristSpecimenPick);
            gripper.setPosition(gripperOpenPosition);
            subWrist.setPosition(subWristUp);
            running = 1;
        }  else if (gamepad2.y) {
            //Drop Sample at high basket
            //increasing the slider timer since zero power behaviour for break is not working.
            sliderTimer = 3000;
            setSliderPosition(SliderExtendPosition);
            wrist.setPosition(wristDropPosition);
            running = 1;
        } else if (gamepad2.x) {
            //Sample Picking Position
            //increasing the slider timer since zero power behaviour for break is not working.
            /*sliderTimer = 5000;
            setSliderPosition(sliderhangposition);
            running = 1;*/
            gripper.setPosition(gripperOpenPosition);
            wrist.setPosition(wristDownPosition);
        } else if (gamepad2.left_bumper) {
            //Pick Sample
            gripper.setPosition(gripperOpenPosition);
        } else if (gamepad2.right_bumper) {
            //Drop Sample
            gripper.setPosition(gripperClosedPosition);
        }else if (gamepad2.dpad_down) {
            subWrist.setPosition(subWristDown);
            subIntake.setPosition(subIntakeIn);
        } else if (gamepad2.dpad_up) {
            subWrist.setPosition(subWristUp);
            //subIntake.setPosition(subIntakeOff);
            subIntake.setPosition(subIntakeIn);
        }  else if (gamepad2.dpad_left) {
            subIntake.setPosition(subIntakeOff);
        } else if (gamepad2.dpad_right) {
            subWrist.setPosition(subWristNeutral);
            subIntake.setPosition(subIntakeOut);
        } else if (gamepad2.back) {
            //gripper_open();
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderSub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //init_variables();
        }else if (gamepad1.dpad_up) {
            // sliderhangposition
            //increasing the slider timer since zero power behaviour for break is not working.
            sliderTimer = 15000;
            setSliderPosition(sliderhangposition);
            wrist.setPosition(wristDropPosition);
        } else if (gamepad1.dpad_down) {
            //Old DPAD DOWN
            setSliderPosition(slider.getCurrentPosition() - 600);
        }else if (gamepad1.a) {
            wrist.setPosition(wristDownPosition);
        }else if (gamepad1.b) {
            wrist.setPosition(wristSpecimenPick);
        }else if (gamepad1.x) {
            wrist.setPosition(wristDropPosition);
        }else if (gamepad1.y) {
            wrist.setPosition(wristUpPosition);
        }/*else if (gamepad1.left_bumper) {
            gripper.setPosition(gripperOpenPosition);
        } else if (gamepad1.right_bumper) {
            gripper.setPosition(gripperClosedPosition);
        }*/
        /*if (gamepad2.left_stick_button) {
            wrist.setPosition(wristUpPosition);
        } else if (gamepad2.right_stick_button) {
            subIntake.setPosition(subIntakeIn);
        }*/
    }
    private void gripper_open() {
        gripper.setPosition(gripperOpenPosition);
    }
}