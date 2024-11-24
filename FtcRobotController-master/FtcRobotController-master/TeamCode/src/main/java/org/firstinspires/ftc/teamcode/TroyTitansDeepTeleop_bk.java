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
    private Servo wrist;
    private DcMotor sliderSub;
    int sliderhangposition;
    int subSliderClosedPosition;
    ElapsedTime runtime;
    int SliderClosePosition;
    int SliderIntakePosition;
    int SliderSubIntakePosition;
    double subIntakeOff;
    int SliderExtendPosition;
    double wristSpecimenPick;
    double subWristNeutral;
    double wristUpPosition;

    double wristDeadBand;
    double wristPower;
    double subWristPower;
    double wristDropPosition;
    double wristDownPosition;
    int gripperClosedPosition;
    int gripperOpenPosition;
    double subIntakeIn;
    float manualArmPower;
    int subIntakeOut;
    double subWristUp;
    double subWristDown;
    boolean manualMode;
    double running;
    int sliderDelay;

    private void init_variables() {
        // Slider Position
        sliderhangposition = 1975;
        // Vertical Slider Position
        SliderClosePosition = 0;
        SliderIntakePosition = 425;
        SliderExtendPosition = 3350;

        SliderSubIntakePosition = 1000;
        // Wrist on Vertical Slider Position
        wristSpecimenPick = 0.5;
        wristUpPosition = 0.85; //Neeed to add a button
        wristDownPosition = 0;  //Neeed to add a button
        wristDropPosition = 0.7;
        wristDeadBand = 0.03;
        wristPower = 0;
        subWristPower = 0;
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
        manualMode = false;
        running = 0;
        runtime = new ElapsedTime();
        sliderDelay = 800;
    }



    // get an instance of the "Robot" class.
    SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override public void runOpMode()
    {
        // Initialize the drive hardware & Turn on telemetry
        robot.initialize(true);

        slider = hardwareMap.get(DcMotor.class, "slider");
        gripper = hardwareMap.get(Servo.class, "gripper");
        subIntake = hardwareMap.get(Servo.class, "subIntake");
        subWrist = hardwareMap.get(Servo.class, "subWrist");
        wrist = hardwareMap.get(Servo.class, "wrist");
        sliderSub = hardwareMap.get(DcMotor.class, "sliderSub");


        init_variables();
        Slider_init();
        SliderSub_init();
        subIntake.setPosition(subIntakeOff);
        subWrist.setPosition(subWristUp);
        wrist.setPosition(wristUpPosition);

        telemetry.addData("Status", "Initialized");
        waitForStart();

        /*
        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            robot.readSensors();
            telemetry.update();
        };
*/
        while (opModeIsActive())
        {
            drive_process();
            gamepadControls();
            runtime.reset();
             /*if (!slider.isBusy()) {
                 slider.setPower(0);
                 telemetry.addData("Slider Is NOT Busy", slider.getCurrentPosition());
                 telemetry.update();
            }*/
            /*while (slider.isBusy() && runtime.milliseconds() <= 1300) {
                // Wait For Slider to Finish
                //slider.setPower(1);
                telemetry.addData("Slider Is Busy", slider.getCurrentPosition(),runtime.milliseconds());
                telemetry.update();
            }*/
            /*while (sliderSub.isBusy() && runtime.milliseconds() <= 600) {
                // Wait For Slider to Finish
                sliderSub.setPower(1);
                telemetry.addData("SliderSub Is Busy", sliderSub.getCurrentPosition());
                telemetry.update();
            }*/
            slider.setPower(0);
            sliderSub.setPower(0);
            Manual_Mode_Slider();
            Manual_Mode_Slidersub();
/*            telemetry.addData("FL Position", frontleft.getCurrentPosition());
            telemetry.addData("FR Position", frontright.getCurrentPosition());
            telemetry.addData("RL Position", rearleft.getCurrentPosition());
            telemetry.addData("RR Position", rearright.getCurrentPosition());*/
            telemetry.addData("Slider Position", slider.getCurrentPosition());
            telemetry.addData("SubSlider Position", sliderSub.getCurrentPosition());
        }
    }

    private void drive_process() {
        robot.readSensors();

        // Allow the driver to reset the gyro by pressing both small gamepad buttons
        if(gamepad1.options && gamepad1.share){
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
            robot.resetHeading();
            robot.resetOdometry();
            //yaw = 180;
            backbutton = true;
        }
        /* else if (gamepad1.dpad_up) {
            drive = SAFE_DRIVE_SPEED / 2.0;
        } else if (gamepad1.dpad_down) {
            drive = -SAFE_STRAFE_SPEED / 2.0;
        }*/

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
            //robot.drive( -15, 0.80, 0.15);
            robot.turnTo(-180, 0.5, 0.25);
            robot.strafe( 60, 0.8, 0);

        }
        else
        {
            //  Drive the wheels based on the desired axis motions
            robot.moveRobot(drive, strafe, yaw);
        }


        // If the robot has just been sitting here for a while, make heading setpoint track any gyro drift to prevent rotating.
        if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
            if (stopTime.time() > HEADING_HOLD_TIME) {
                robot.yawController.reset(robot.getHeading());  // just keep tracking the current heading
            }
        } else {
            stopTime.reset();
        }
    }

    private void Slider_init() {
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotor.Direction.FORWARD);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setPower(0);
    }

    private void SliderSub_init() {
        sliderSub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderSub.setDirection(DcMotor.Direction.FORWARD);
        sliderSub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderSub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while (gamepad2.left_stick_y != 0 && runtime.milliseconds() <= 2000) {
            //if (gamepad2.left_stick_y > 0) {
            slider.setPower(-gamepad2.left_stick_y);
         /*   } else {
                break;
            }*/
        }
       /* while (gamepad2.left_stick_y < 0 && runtime.milliseconds() <= 2000) {
            if (gamepad2.left_stick_y < 0) {
                slider.setPower(Math.abs(gamepad2.left_stick_y));
            } else {
                break;
            }
        }*/
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Slider Manual Mode Allowed", slider.getCurrentPosition());
        telemetry.update();
        slider.setPower(0);
    }

    private void Manual_Mode_Slidersub() {
        sliderSub.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while (gamepad2.right_stick_y != 0 && runtime.milliseconds() <= 1000) {
            gripper.setPosition(gripperClosedPosition);
            wrist.setPosition(wristUpPosition);
            sliderSub.setPower(-gamepad2.right_stick_y);
        }

        sliderSub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderSub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Slider Manual Mode Allowed", sliderSub.getCurrentPosition());
        telemetry.update();
        sliderSub.setPower(0);
    }

    private void gamepadControls() {
        wristPower = gamepad2.right_trigger - gamepad2.left_trigger;
        if(wristPower > wristDeadBand)
            wrist.setPosition(wristPower);

        subWristPower = gamepad1.right_trigger - gamepad1.left_trigger;
        if(subWristPower > wristDeadBand)
            subWrist.setPosition(subWristPower);

        if (gamepad2.a) {
            //Home position
            robot.stopRobot();
            slider.setTargetPosition(SliderClosePosition);
            slider.setPower(1);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPosition(wristUpPosition);
            gripper.setPosition(gripperClosedPosition);
            // Retract Horizontal Submersible Slider
            sliderSub.setTargetPosition(subSliderClosedPosition);
            sliderSub.setPower(1);
            sliderSub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(sliderDelay);
            // Wrist on Submersible Up
            subWrist.setPosition(subWristUp);
            running = 1;
        } else if (gamepad2.b) {
            //Specimen Pick Positions
            robot.stopRobot();
            // Horizontal Submersible Slider close to specimen Pick
            subSlider_homePosition();
            // Vertical Slider Up and Away from Horozontal Slider
            slider.setTargetPosition(SliderIntakePosition);
            slider.setPower(1);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(sliderDelay);
            wrist.setPosition(wristSpecimenPick);
            gripper.setPosition(gripperOpenPosition);
            subWrist.setPosition(subWristUp);
            running = 1;
        }  else if (gamepad2.y) {
            //Drop Sample at high basket
            robot.stopRobot();
            wrist.setPosition(wristDropPosition);
            //subSlider_noFallPosition();
            slider.setTargetPosition(SliderExtendPosition);
            slider.setPower(1);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2*sliderDelay);
            running = 1;
        } else if (gamepad2.x) {
            //Sample Picking Position
            // gripper.setPosition(gripperClosedPosition);
            robot.stopRobot();
            /*subIntake.setPosition(subIntakeIn);
            subWrist.setPosition(subWristUp);*/
            slider.setTargetPosition(sliderhangposition);
            slider.setPower(1);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(sliderDelay);
            running = 1;
        } else if (gamepad2.left_bumper) {
            //Pick Sample
            //subIntake.setPosition(subIntakeIn);
            gripper.setPosition(gripperOpenPosition);
        } else if (gamepad2.right_bumper) {
            //Drop Sample
            // subIntake.setPosition(subIntakeOut);
            gripper.setPosition(gripperClosedPosition);
        }else if (gamepad2.dpad_down) {
            subWrist.setPosition(subWristDown);
            subIntake.setPosition(subIntakeIn);
        } else if (gamepad2.dpad_up) {
            subWrist.setPosition(subWristUp);
            subIntake.setPosition(subIntakeOff);
        }  else if (gamepad2.dpad_left) {
            subIntake.setPosition(subIntakeOff);
        } else if (gamepad2.dpad_right) {
            subWrist.setPosition(subWristNeutral);
            subIntake.setPosition(subIntakeOut);
        } else if (gamepad2.back) {
            gripper_open();
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderSub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            init_variables();
        }
        /*else if (gamepad1.b) {
            gripper.setPosition(gripperClosedPosition);
            slider.setTargetPosition(slider.getCurrentPosition() + 600);
            slider.setPower(1);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(sliderDelay);
            //wrist.setPosition(wristUpPosition);
        } */else if (gamepad1.dpad_up) {
            // sliderhangposition
           /* gripper.setPosition(gripperClosedPosition);
            sleep(1000);*/
            robot.stopRobot();
            slider.setTargetPosition(sliderhangposition);
            slider.setPower(1);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(sliderDelay);
            wrist.setPosition(wristSpecimenPick);
        } else if (gamepad1.dpad_down) {
            //Old DPAD DOWN
            robot.stopRobot();
            slider.setTargetPosition(slider.getCurrentPosition() - 600);
            slider.setPower(1);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //gripper.setPosition(gripperOpenPosition);
            sleep(sliderDelay/2);
        }else if (gamepad1.a) {
            wrist.setPosition(wristDownPosition);
        }else if (gamepad1.b) {
            wrist.setPosition(wristSpecimenPick);
        }else if (gamepad1.x) {
            wrist.setPosition(wristDropPosition);
        }else if (gamepad1.y) {
            wrist.setPosition(wristUpPosition);
        }else if (gamepad1.left_bumper) {
            gripper.setPosition(gripperOpenPosition);
        } else if (gamepad1.right_bumper) {
            gripper.setPosition(gripperClosedPosition);
        }/*else if (gamepad1.dpad_down) {
            wrist.setPosition(wristDownPosition);
        } else if (gamepad1.dpad_up) {
            wrist.setPosition(wristUpPosition);
        }*/
        if (gamepad2.left_stick_button) {
            wrist.setPosition(wristUpPosition);
        } else if (gamepad2.right_stick_button) {
            subIntake.setPosition(subIntakeIn);
        }



    }
    private void gripper_open() {
        gripper.setPosition(gripperOpenPosition);
    }
}