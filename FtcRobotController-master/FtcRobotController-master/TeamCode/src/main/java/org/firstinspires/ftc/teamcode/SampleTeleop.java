/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    final double SAFE_DRIVE_SPEED   = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
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
    double subIntakeOff;
    int SliderExtendPosition;
    double wristSpecimenPick;
    double subWristNeutral;
    int wristUpPosition;
    int wristDownPosition;
    int gripperClosedPosition;
    int gripperOpenPosition;
    int subIntakeIn;
    float manualArmPower;
    int subIntakeOut;
    int subWristUp;
    int subWristDown;
    boolean manualMode;
    int running;

    private void init_variables() {
        // Slider Position
        sliderhangposition = 950;
        // Vertical Slider Position
        SliderClosePosition = 0;
        SliderIntakePosition = 450;
        SliderExtendPosition = 3450;
        // Wrist on Vertical Slider Position
        wristSpecimenPick = 0.5;
        wristUpPosition = 0;
        wristDownPosition = 1;
        // Gripper on Vertical Slider Position
        gripperClosedPosition = -1;
        gripperOpenPosition = 1;
        // Horizontal Submersible Slider Position
        subSliderClosedPosition = 0;
        // Intake on Submersible Slider Position
        subIntakeOff = 0.5;
        subIntakeIn = -1;
        subIntakeOut = 1;
        // Wrist on Submersible Slider Position
        subWristNeutral = 0.5;
        subWristDown = 1;
        subWristUp = 0;
        // Initiation
        manualMode = false;
        running = 0;
        runtime = new ElapsedTime();
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
        Silder_init();
        SilderSub_init();
        subIntake.setPosition(subIntakeOff);
        subWrist.setPosition(subWristNeutral);

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
            if (!slider.isBusy()) {
                slider.setPower(0);
            }
            while (slider.isBusy() && runtime.milliseconds() <= 1300) {
                // Wait For Slider to Finish
                slider.setPower(1);
                telemetry.addData("Slider Is Busy", slider.getCurrentPosition());
                telemetry.update();
            }
            while (sliderSub.isBusy() && runtime.milliseconds() <= 600) {
                // Wait For Slider to Finish
                sliderSub.setPower(1);
                telemetry.addData("SliderSub Is Busy", sliderSub.getCurrentPosition());
                telemetry.update();
            }
            slider.setPower(0);
            sliderSub.setPower(0);
            Manual_Mode_Slider();
            Manual_Mode_Slidersub();
            telemetry.addData("FL Position", frontleft.getCurrentPosition());
            telemetry.addData("FR Position", frontright.getCurrentPosition());
            telemetry.addData("RL Position", rearleft.getCurrentPosition());
            telemetry.addData("RR Position", rearright.getCurrentPosition());
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

        //  OR... For special conditions, Use the DPAD to make pure orthoginal motions
        if (gamepad1.dpad_left) {
            strafe = SAFE_DRIVE_SPEED / 2.0;
        } else if (gamepad1.dpad_right) {
            strafe = -SAFE_DRIVE_SPEED / 2.0;
        } else if (gamepad1.dpad_up) {
            drive = SAFE_DRIVE_SPEED / 2.0;
        } else if (gamepad1.dpad_down) {
            drive = -SAFE_STRAFE_SPEED / 2.0;
        }

        // This is where we heep the robot heading locked so it doesn't turn while driving or strafing in a straight line.
        // Is the driver turning the robot, or should it hold its heading?
        if (Math.abs(yaw) > 0.05) {
            // driver is commanding robot to turn, so turn off auto heading.
            autoHeading = false;
        } else {
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

        //  Drive the wheels based on the desired axis motions
        robot.moveRobot(drive, strafe, yaw);

        // If the robot has just been sitting here for a while, make heading setpoint track any gyro drift to prevent rotating.
        if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
            if (stopTime.time() > HEADING_HOLD_TIME) {
                robot.yawController.reset(robot.getHeading());  // just keep tracking the current heading
            }
        } else {
            stopTime.reset();
        }
    }

    private void Silder_init() {
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotor.Direction.FORWARD);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void SilderSub_init() {
        sliderSub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderSub.setDirection(DcMotor.Direction.FORWARD);
        sliderSub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderSub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void subSlider_homePosition() {
        sliderSub.setTargetPosition(subSliderClosedPosition);
        sliderSub.setPower(1);
        sliderSub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        subIntake.setPosition(subIntakeOff);
    }

    private void Manual_Mode_Slider() {
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while (gamepad2.left_stick_y > 0 && runtime.milliseconds() <= 2000) {
            if (gamepad2.left_stick_y > 0) {
                slider.setPower(-gamepad2.left_stick_y);
            } else {
                break;
            }
        }
        while (gamepad2.left_stick_y < 0 && runtime.milliseconds() <= 2000) {
            if (gamepad2.left_stick_y < 0) {
                slider.setPower(Math.abs(gamepad2.left_stick_y));
            } else {
                break;
            }
        }
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Slider Manual Mode Allowed", slider.getCurrentPosition());
        telemetry.update();
        slider.setPower(0);
    }

    private void Manual_Mode_Slidersub() {
        sliderSub.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while (gamepad2.right_stick_y > 0 && runtime.milliseconds() <= 1000) {
            gripper.setPosition(gripperClosedPosition);
            wrist.setPosition(wristUpPosition);
            if (gamepad2.right_stick_y > 0) {
                sliderSub.setPower(-gamepad2.right_stick_y);
            } else {
                break;
            }
        }
        while (gamepad2.right_stick_y < 0 && runtime.milliseconds() <= 1000) {
            gripper.setPosition(gripperClosedPosition);
            wrist.setPosition(wristUpPosition);
            if (gamepad2.right_stick_y < 0) {
                sliderSub.setPower(Math.abs(gamepad2.right_stick_y));
            } else {
                break;
            }
        }
        sliderSub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderSub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Slider Manual Mode Allowed", sliderSub.getCurrentPosition());
        telemetry.update();
        sliderSub.setPower(0);
    }

    private void gamepadControls() {
        if (gamepad2.a) {
            //manualArmPower = gamepad2.right_trigger - gamepad2.left_trigger;
            drive_zero();
            gripper_open();
            slider.setTargetPosition(SliderClosePosition);
            slider.setPower(1);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPosition(wristDownPosition);
            gripper.setPosition(gripperClosedPosition);
            // Retract Horizontal Submersible Slider
            sliderSub.setTargetPosition(subSliderClosedPosition);
            sliderSub.setPower(1);
            sliderSub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Wrist on Submersible Up
            subWrist.setPosition(subWristUp);
            running = 1;
        } else if (gamepad2.b) {
            drive_zero();
            // Horizontal Submersible Slider close to specimen Pick
            subSlider_homePosition();
            // Vertical Slider Up and Away from Horozontal Slider
            slider.setTargetPosition(SliderIntakePosition);
            slider.setPower(1);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPosition(wristUpPosition);
            gripper.setPosition(gripperClosedPosition);
            subWrist.setPosition(subWristDown);
            running = 1;
        }  else if (gamepad2.y) {
            drive_zero();
            subSlider_homePosition();
            slider.setTargetPosition(SliderExtendPosition);
            slider.setPower(1);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPosition(wristUpPosition);
            running = 1;
        } else if (gamepad2.x) {
            drive_zero();
            subIntake.setPosition(1);
            subWrist.setPosition(0.5);
            sliderSub.setTargetPosition(SliderIntakePosition);
            sliderSub.setPower(1);
            sliderSub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            running = 1;
        }  else if (gamepad2.left_trigger) {
            subIntake.setPosition(subIntakeIn);
        } else if (gamepad2.right_trigger) {
            subIntake.setPosition(subIntakeOut);
        }else if (gamepad2.left_bumper) {
            subWrist.setPosition(subWristDown);
        } else if (gamepad2.right_bumper) {
            subWrist.setPosition(subWristUp);
        }else if (gamepad2.dpad_right) {
            subWrist.setPosition(subWristNeutral);
        }  else if (gamepad2.dpad_up) {
            wrist.setPosition(wristUpPosition);
        } else if (gamepad2.dpad_down) {
            wrist.setPosition(wristDownPosition);
        } else if (gamepad2.dpad_left) {
            wrist.setPosition(wristSpecimenPick);
        }else if (gamepad2.back) {
            gripper_open();
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderSub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            init_variables();
        } else if (gamepad1.left_trigger) {
            gripper.setPosition(gripperOpenPosition);
        }else if (gamepad1.right_trigger) {
            gripper.setPosition(gripperClosedPosition);
        }else if (gamepad1.a) {
            subIntake.setPosition(subIntakeOff);
        } else if (gamepad1.dpad_up) {
            wrist.setPosition(wristUpPosition);
        } else if (gamepad1.dpad_down) {
            wrist.setPosition(wristDownPosition);
        } else if (gamepad1.dpad_left) {
            wrist.setPosition(wristSpecimenPick);
        }
        /*else if (gamepad1.b) {
            subIntake.setPosition(subIntakeIn);
        } else if (gamepad1.y) {
            subIntake.setPosition(subIntakeOut);
        } */


       /* if (gamepad2.left_stick_button) {
            wrist.setPosition(wristDownPosition);
        } else if (gamepad2.right_stick_button) {
            wrist.setPosition(wristUpPosition);
        }*/


    }
    private void drive_zero() {
        frontleft.setPower(0);
        frontright.setPower(0);
        rearleft.setPower(0);
        rearright.setPower(0);
    }

}