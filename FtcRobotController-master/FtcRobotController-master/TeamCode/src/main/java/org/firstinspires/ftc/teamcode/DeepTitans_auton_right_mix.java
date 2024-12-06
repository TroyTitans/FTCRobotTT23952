/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="DeepTroyTitansAutonRightMix", group = "TroyTitans", preselectTeleOp = "TroyTitansDeepTeleop")
public class DeepTroyTitansAutonRightMix extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    private DcMotor slider;
    private Servo gripper;
    private TouchSensor touchslider;
    private Servo wrist;
    private Servo subWrist;
    private DcMotor sliderSub;
    private Servo subIntake;

    double sliderhangposition;
    double SliderIntakePosition;;
    double sliderHighbasket;

    double sliderHangSpec;
    double sliderMovePos;

    double wristUpPosition;
    double wristDownPosition;
    double wristScoringPosition;
    double wristHangSpecPosition;

    double subIntakeOff;
    double subWristUp;

    double gripperClosedPosition;
    double gripperOpenPosition;

    double drivePower;
    double driveForward;
    double strafePower;

    private void init_variables() {
        // Slider Position
        sliderhangposition = 1400;
        // Vertical Slider Position
        SliderIntakePosition = 475;
        sliderHangSpec = 2400;
        sliderMovePos = 1000;

        wristUpPosition = 0.85;
        wristDownPosition = 0;
        wristScoringPosition = 0.7;
        wristHangSpecPosition = 0.5;
        // wristSpecimenPick = 0.5;

        gripperClosedPosition = 1;
        gripperOpenPosition = 0;

        subIntakeOff = 0.5;
        subWristUp = 0;
        drivePower = 0.6;
        driveForward = 0.4;
        strafePower = 0.6;


    }


    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true,true);
        slider = hardwareMap.get(DcMotor.class, "slider");
        gripper = hardwareMap.get(Servo.class, "gripper");
        subIntake = hardwareMap.get(Servo.class, "subIntake");
        subWrist = hardwareMap.get(Servo.class, "subWrist");
        wrist = hardwareMap.get(Servo.class, "wrist");
        sliderSub = hardwareMap.get(DcMotor.class, "sliderSub");
        touchslider = hardwareMap.get(TouchSensor.class, "touchslider");


        init_variables();
        Slider_init();
        SliderSub_init();
        subIntake.setPosition(subIntakeOff);
        subWrist.setPosition(subWristUp);
        wrist.setPosition(wristUpPosition);
        gripper.setPosition(gripperOpenPosition);

        sleep(3000);
        gripper.setPosition(gripperClosedPosition);

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            Autonomus();
        }
    }


      /*  private void setupForPickupSample() {
            wrist.setPosition(wristDownPosition);
            setSliderPosition(sliderSamplePick);
            sleep(500);
        }*/

      /*  private void pickSample() {
            gripper.setPosition(gripperClosedPosition);
            sleep(200);
            wrist.setPosition(wristUpPosition);
        }*/
      /*  private void DropSample() {
            setSliderPosition(sliderHighbasket);
            sleep(2000);
            robot.drive( 5, 0.50, 0.01);
            wrist.setPosition(wristScoringPosition);
            gripper.setPosition(gripperOpenPosition);
            sleep(500);
        }*/

    private void Autonomus() {
        // Set slider and wrist to hang position


        //Drive to sub to hang in high rung
        setSliderPosition(sliderhangposition);
        wrist.setPosition(wristScoringPosition);
        //  sleep(300);
        robot.drive(  37.5, 0.30, 0);
    /*
            robot.drive(  6, 0.30, 0);*/
        setSliderPosition(sliderHangSpec);
        sleep(500);
        gripper.setPosition(gripperOpenPosition);
        sleep(500);

        //reverse and set up for sample picking
        // robot.drive(  -5, 0.80, 0.20);
        //setSliderPosition(sliderSamplePick);
    /*        setSliderPosition(SliderIntakePosition);
            wrist.setPosition(wristUpPosition);
            gripper.setPosition(gripperOpenPosition);*/


        wrist.setPosition(wristUpPosition);
        gripper.setPosition(gripperOpenPosition);

        //move left to pick first sample
        robot.strafe( -40, strafePower, 0);
        setSliderPosition(-2500);
        robot.drive( 37, drivePower, 0);
        robot.strafe( -13, strafePower, 0);
        robot.drive( -57, drivePower, 0);


        //Push second piece


        robot.drive( 57, drivePower, 0);
        wrist.setPosition(wristHangSpecPosition);
        robot.turnTo(180, 0.45, 0);
        if (touchslider.isPressed())
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setSliderPosition(SliderIntakePosition);
        robot.strafe( 12, strafePower, 0);

        robot.drive( 65, driveForward, 0);
        sleep(500);
        gripper.setPosition(gripperClosedPosition);
        sleep(300);

        //Hang second specimen
        setSliderPosition(sliderhangposition+50);
        wrist.setPosition(wristScoringPosition);
        robot.strafe( -67, strafePower, 0);
        robot.turnTo(0, 0.45, 0);
        robot.drive( 30, drivePower, 0);
        gripper.setPosition(gripperOpenPosition);
        sleep(500);

        setSliderPosition(sliderMovePos);
        robot.drive( -7, drivePower, 0);


        //pick third specimen
        //robot.turnTo(90, 0.45, 0);
        robot.turnTo(180, 0.45, 0);
        setSliderPosition(SliderIntakePosition);
        wrist.setPosition(wristHangSpecPosition);
        gripper.setPosition(gripperOpenPosition);
        //robot.drive( 15, drivePower, 0);

        //robot.specimenMove(29.7,55,0.3,0);

        robot.strafe( 55, strafePower, 0);
        robot.drive( 29.7, driveForward, 0);


        sleep(1000);
        gripper.setPosition(gripperClosedPosition);
        sleep(300);

        //Hang third specimen

        setSliderPosition(sliderhangposition+50);
        wrist.setPosition(wristScoringPosition);
        robot.turnTo(0, 0.45, 0);
        robot.strafe( 48, strafePower, 0);
        robot.drive( 31, drivePower, 0);
        gripper.setPosition(gripperOpenPosition);
        sleep(500);
        setSliderPosition(sliderMovePos);

        //Park
        robot.specimenMove(-30,-50,strafePower,0);
        setSliderPosition(0);
        wrist.setPosition(wristUpPosition);
        gripper.setPosition(gripperClosedPosition);


        requestOpModeStop();
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
        sliderSub.setPower(0);
    }

    private void setSliderPosition(double sliderPosition) {
        slider.setTargetPosition((int)sliderPosition);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
