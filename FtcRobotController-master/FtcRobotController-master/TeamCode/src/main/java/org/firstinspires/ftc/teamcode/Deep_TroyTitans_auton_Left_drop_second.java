package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Deep_GoBilda_Teleop_working (Blocks to Java)")
public class Deep_GoBilda_Teleop_working extends LinearOpMode {

  private DcMotor armLeft;
  private DcMotor armRight;
  private DcMotor slider;
  private Servo gripper;
  private Servo subIntake;
  private Servo subWrist;
  private Servo wrist;
  private DcMotor sliderSub;
  private DcMotor frontleft;
  private DcMotor frontright;
  private DcMotor rearleft;
  private DcMotor rearright;

  float z;
  float x;
  float y;

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

  /**
   * Describe this function...
   */
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

  /**
   * Describe this function...
   */
    /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    slider = hardwareMap.get(DcMotor.class, "slider");
    gripper = hardwareMap.get(Servo.class, "gripper");
    subIntake = hardwareMap.get(Servo.class, "subIntake");
    subWrist = hardwareMap.get(Servo.class, "subWrist");
    wrist = hardwareMap.get(Servo.class, "wrist");
    sliderSub = hardwareMap.get(DcMotor.class, "sliderSub");

    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    rearleft = hardwareMap.get(DcMotor.class, "rearleft");
    rearright = hardwareMap.get(DcMotor.class, "rearright");

    // You will need to reverse one of the drive motors.
    init_variables();
    drive_init();
    Silder_init();
    SilderSub_init();
    subIntake.setPosition(subIntakeOff);
    subWrist.setPosition(subWristNeutral);
    telemetry.addData("Status", "Initialized");
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Show the motor's status via telemetry
        // Split Arcade Drive
        // Configure Slider to Run to Position
        // Control the drive motors
        drive_process();
        if (gamepad2.a) {
          manualArmPower = gamepad2.right_trigger - gamepad2.left_trigger;
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
        } else if (gamepad1.dpad_up) {
          wrist.setPosition(wristUpPosition);
        } else if (gamepad1.dpad_down) {
          wrist.setPosition(wristDownPosition);
        } else if (gamepad1.dpad_right) {
          wrist.setPosition(wristSpecimenPick);
        } else if (gamepad1.dpad_left) {
        } else if (gamepad2.y) {
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
        } else if (gamepad1.a) {
          subIntake.setPosition(subIntakeOff);
        } else if (gamepad1.b) {
          subIntake.setPosition(subIntakeIn);
        } else if (gamepad1.y) {
          subIntake.setPosition(subIntakeOut);
        } else {
          if (gamepad2.dpad_right) {
            subWrist.setPosition(subWristNeutral);
          } else if (gamepad2.dpad_up) {
            subWrist.setPosition(subWristUp);
          } else if (gamepad2.dpad_down) {
            subWrist.setPosition(subWristDown);
          } else if (gamepad2.back) {
            gripper_open();
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderSub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            init_variables();
          } else if (gamepad1.dpad_up) {
            drive_zero();
            hang_position();
          } else {
            if (gamepad1.dpad_down) {
              drive_zero();
              hang_specimen();
            }
          }
        }
        // Re-zero Encoder Button
        // Watchdog to shut down motor once arm reaches the home position
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
        // Gripper
        if (gamepad2.left_bumper) {
          gripper.setPosition(gripperOpenPosition);
          telemetry.update();
        }
        // Gripper2
        if (gamepad2.right_bumper) {
          gripper.setPosition(gripperClosedPosition);
        }
        if (gamepad2.left_stick_button) {
          wrist.setPosition(wristDownPosition);
        } else if (gamepad2.right_stick_button) {
          wrist.setPosition(wristUpPosition);
        }
        Manual_Mode_Slider();
        Manual_Mode_Slidersub();
        telemetry.addData("FL Position", frontleft.getCurrentPosition());
        telemetry.addData("FR Position", frontright.getCurrentPosition());
        telemetry.addData("RL Position", rearleft.getCurrentPosition());
        telemetry.addData("RR Position", rearright.getCurrentPosition());
        telemetry.addData("Slider Position", slider.getCurrentPosition());
        telemetry.addData("SubSlider Position", sliderSub.getCurrentPosition());
        // Set Slider for Manual
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Silder_init() {
    slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slider.setDirection(DcMotor.Direction.FORWARD);
    slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  /**
   * Describe this function...
   */
  private void SilderSub_init() {
    sliderSub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    sliderSub.setDirection(DcMotor.Direction.FORWARD);
    sliderSub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    sliderSub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  /**
   * Describe this function...
   */
  private void subSlider_homePosition() {
    sliderSub.setTargetPosition(subSliderClosedPosition);
    sliderSub.setPower(1);
    sliderSub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    subIntake.setPosition(subIntakeOff);
  }

   /**
   * Describe this function...
   */
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
      if (gamepad2.right_stick_y > 0) {
        sliderSub.setPower(-gamepad2.right_stick_y);
      } else {
        break;
      }
    }
    while (gamepad2.right_stick_y < 0 && runtime.milliseconds() <= 1000) {
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

  /**
   * Describe this function...
   */
  private void hang_position() {
  }

  /**
   * Describe this function...
   */
  private void hang_specimen() {
  }
}
