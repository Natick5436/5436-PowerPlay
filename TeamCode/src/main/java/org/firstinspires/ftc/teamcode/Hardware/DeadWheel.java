package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
public class DeadWheel {

    private double ticksPer;
    private double wheelCirc;
    private DcMotor device;
    private int direction;
    private DcMotor.Direction motorDirection;
    private double deadWheelCorrection;

    public DeadWheel(DcMotor configMotor, int direction){
        this.motorDirection = configMotor.getDirection();
        if(motorDirection == DcMotor.Direction.REVERSE){
            this.direction = -direction;
        }else {
            this.direction = direction;
        }
        device = configMotor;
        ticksPer = 1;
        wheelCirc = 0;
        deadWheelCorrection = 1;
    }
    public DeadWheel(DcMotor configMotor, double wheelDiameter, double ticksPer, int direction){
        this.motorDirection = configMotor.getDirection();
        if(motorDirection == DcMotor.Direction.REVERSE){
            this.direction = -direction;
        }else {
            this.direction = direction;
        }
        device = configMotor;
        this.wheelCirc = wheelDiameter*Math.PI;
        this.ticksPer = ticksPer;
        deadWheelCorrection = 1;
    }
    public DeadWheel(DcMotor configMotor, double wheelDiameter, double ticksPer, int direction, double deadWheelCorrection){
        this.motorDirection = configMotor.getDirection();
        if(motorDirection == DcMotor.Direction.REVERSE){
            this.direction = -direction;
        }else {
            this.direction = direction;
        }
        device = configMotor;
        this.wheelCirc = wheelDiameter*Math.PI;
        this.ticksPer = ticksPer;
        this.deadWheelCorrection = deadWheelCorrection;
    }

    public void updateParameters(){
        if(motorDirection != device.getDirection()){
            direction = -direction;
            motorDirection = device.getDirection();
        }
    }
    public void setDirection(int d) {
        if(motorDirection == DcMotor.Direction.REVERSE) {
            direction = -d;
        }else{
            direction = d;
        }
    }
    public int getCurrentPosition(){
        updateParameters();
        return direction*device.getCurrentPosition();
    }
    public double distanceTraveled(){
        return deadWheelCorrection*this.wheelCirc*getCurrentPosition()/this.ticksPer;
    }

    public double getWheelCirc() {
        return wheelCirc;
    }

    public void setWheelCirc(double wheelCirc) {
        this.wheelCirc = wheelCirc;
    }

    public double getTicksPerRev() {
        return ticksPer;
    }

    public void setTicksPerRev(double ticksPer) {
        this.ticksPer = ticksPer;
    }
}
