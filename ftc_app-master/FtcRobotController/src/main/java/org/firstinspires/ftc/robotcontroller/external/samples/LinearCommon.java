package org.firstinspires.ftc.robotcontroller.external.samples;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ColorSensor", group="Sensor")
public class LinearCommon extends LinearOpMode {
    // Declare OpMode members.
    HardwarePushbot robot = new HardwarePushbot();

    //public final static double COLORWRIST_MIN_RANGE = 0.0;
    //public final static double COLORWRIST_MAX_RANGE = 0.5;

    protected Servo ColorArmServo = null;
    protected Servo ColorWristServo = null;
    protected DcMotor leftDc = null;
    protected DcMotor rightDC = null;
    protected ColorSensor color;
    @Override
    public void runOpMode() {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        boolean bLedOn = true;

        color = hardwareMap.colorSensor.get("ColorSensor");
        ColorArmServo = hardwareMap.servo.get("ColorArmServo");
        ColorWristServo = hardwareMap.servo.get("ColorWristServo");

        leftDc = hardwareMap.get(DcMotor.class, "leftDc");
        leftDc.setDirection(DcMotor.Direction.REVERSE);
        leftDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightDC = hardwareMap.get(DcMotor.class, "rightDC");
        rightDC.setDirection(DcMotor.Direction.REVERSE);
        rightDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        color.enableLed(bLedOn);

        telemetry.addData("Initialization", "Complete");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            positionInbetweenJewel();
            if (isBlueSensed()) {
                swipeTowardsBlue();
            } else if (isRedSensed()) {
                swipeTowardsRed();
            } else {
                ColorWristServo.setPosition(0.85);
                sleep(5000);
                ColorArmServo.setPosition(0);
            }
            moveForward();
        }

        rightDC.setPower(0);
        leftDc.setPower(0);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        telemetry.update();

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }
    }

    protected void swipeTowardsBlue() {

    }

    protected void swipeTowardsRed() {

    }

    private void moveForward() {
        sleep(1000);
        leftDc.setTargetPosition(4800);
        rightDC.setTargetPosition(4800);
        leftDc.setPower(0.5);
        rightDC.setPower(0.65);
        sleep(30000000);
    }

    private void positionInbetweenJewel() {
        ColorArmServo.setPosition(0.1);
        sleep(2000);
        ColorWristServo.setPosition(0.35);
        sleep(2000);
        ColorArmServo.setPosition(0.7);
        sleep(4000);
    }

    protected boolean isBlueSensed() {
        return color.blue() > color.red() && color.blue() > color.green();
    }

    protected boolean isRedSensed() {
        return color.red() > color.blue() && color.red() > color.green();
    }
}
