package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class Arm {

    public enum StateArm {
        FRONT,
        TOP,
        MIDDLE,
        BOTTOM,
        IDLE
    }

    public enum StateHand {
        OPEN,
        CLOSED
    }

    public StateArm armState;
    private StateArm lastArmState, previousArmState;
    private ElapsedTime timer;
    public StateHand handState;
    public DcMotorEx motor;
    private Servo left;
    private Servo right;
    public int motorOffset, zeroOffset;


    public Arm(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "Arm");
        left = hardwareMap.get(Servo.class, "leftHand");
        right = hardwareMap.get(Servo.class, "rightHand");
        motorOffset = 0;
        zeroOffset = -920;
        armState = StateArm.FRONT;
        handState = StateHand.OPEN;
        timer = new ElapsedTime();
        previousArmState = StateArm.FRONT;
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void nextPos() {
        switch(armState) {
            case TOP:
                armState = StateArm.FRONT;
                break;
            case MIDDLE:
                armState = StateArm.TOP;
                break;
            case BOTTOM:
                armState = StateArm.MIDDLE;
                break;
        }
    }

    public void previousPos() {
        switch(armState) {
            case FRONT:
                armState = StateArm.TOP;
                break;
            case TOP:
                armState = StateArm.MIDDLE;
                break;
            case MIDDLE:
                armState = StateArm.BOTTOM;
                break;
        }
    }

    public void toggleHand() {
        switch(handState) {
            case OPEN:
                handState = StateHand.CLOSED;
                break;
            case CLOSED:
                handState = StateHand.OPEN;
        }
    }
    public void forceReset() {
        motorOffset = motor.getCurrentPosition() + zeroOffset;
    }

    public void update() {
        if(armState != lastArmState) {
            previousArmState = lastArmState;
            lastArmState = armState;
            timer.reset();
        }
        if (timer.milliseconds() > 750 && armState == StateArm.IDLE && previousArmState == StateArm.FRONT) { //TODO: only armState Front
            forceReset();
        }

        switch (armState) {
            case FRONT:
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setTargetPosition(motorOffset + 917);
                motor.setPower(0.45);
                break;
            case TOP:
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setTargetPosition(motorOffset + 294);
                motor.setPower(0.45);
                break;
            case MIDDLE:
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setTargetPosition(motorOffset + 167);
                motor.setPower(0.45);
                break;
            case BOTTOM:
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setTargetPosition(motorOffset + 43);
                motor.setPower(0.45);
                break;
            case IDLE:
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor.setPower(0);
        }
        if(motor.getCurrentPosition() > 700 && armState == StateArm.FRONT) {
            motor.setPower(0);
        }

        switch (handState) {
            case OPEN:
                left.setPosition(0.5);
                right.setPosition(0.2);
                break;
            case CLOSED:
                left.setPosition(0.55);
                right.setPosition(0.165);
                break;
        }

    }

}
