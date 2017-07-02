## Effect each of the P, I, D components had in implementation

### P Error = Kp * CTE(Cross track error)
   Kp is main factor to determine the steering angle.  Kp value determine how fast the car can turn.  But use only Kp for steering angle will overshoot and cause oscillation.
   
### D Error = Kd * (CTE - Previous CTE) /(delta time)
   Kd is used to counter the steering angle to prevent oscillation. If delta time is fixed, it can be combined in Kd value.
   
### I Error = Ki * (sum of CTE)
   Ki is used to fix the offset of CTE.  This value can make car more stable.

### Video for PID controller 
   A PID controller implementation at speed 30 in the following video.
   https://youtu.be/9l6mCjQfYYU   

## Hyper parameters Tuning Process

**Throttle PID**

To set a steady speed, another throttle PID is used to control the speed of car.  The cross track error(CTE) become the different between current speed and 'set speed'. Also a little brake is implemented if cross track error for steering is large.  
```c   
   double throttle_error = set_speed - speed - fabs(cte) * 3.5;
```   

**Steering PID**

   I use both manual and twiddle to tune the hyperparameters of steering PID.
   
**Manul Tuning of Steering PID**

1. Increase Kp to be able to pass every sharp turns
   Start from zeroes, increase Kp to correct the cross track error(CTE).  
2. Increase Kd to stop oscillation 
  Kd will counter the effect of oscillation with increase of Kp. But with addition of Kd, steering angle might be not enough. 
3. Icrease both Kp and Kd gradually until the car can pass every sharp turns.
4. Increase Ki to correct offset of CTE.  This will also stablilze the car.  But if too big value will cause further oscillation again.

**Twiddle for Optimization**

After I get initial values from manual tuning, I use twiddle method to optimize parameters.  I use the simulator reset command after each test so that I can use the class 'TRAINING' to tune automatically.  The process is slow because I need to finish a whole circle for calculate errors. 

***Parameter value reduce and expand factors***

I change the reduce factor **0.9** to **0.5** and expand factor **1.1** to **1.5** in order to speed up the process.  

***Manually adjust after twiddle***

After tiddle finished, I still manually adjust Kd(increment) so that the car looks more stable.

**Different Speed**

The original set speed is 30 for all the tuning process.  If set higher speed, like 50, the Kd factor need to increase quick a bit.  In our case, Kd is -4.9(at speed 30) and -21(at speed 50) in order to stablize the car from oscillation. The Kp factor need to increase just a little bit to pass all turns, probably because the increment of Kd.  
