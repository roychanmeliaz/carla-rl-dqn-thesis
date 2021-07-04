# # Carla RL-DQN Thesis Project

A bachelor thesis project about **autonomous car maneuver** around **roundabout** using **reinforcement learning** with **deep q network**.

## Environment Setup

Two kinds of roundabouts are used in this research.
 - **Roundabout with Intersection**   
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/bundaran.png?raw=true" alt="roundabout" width="400">

 - **Roundabout without Intersection**   
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/bundaran_tanpa_simpang.png?raw=true" alt="roundabout2" width="400">

## Sensor
The camera sensor was obtained, and then we did the semantic segmentation using CARLA's built-in function.   
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/rgb.png?raw=true" alt="rgb" height="100"> -> 
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/segmentasi.png?raw=true" alt="segmentasi" height="100">
>RGB -> Semantic Segmentation

Two kinds of camera setup are used in this research.


 - **Segmented and Grayscaled**
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/grayscale.png?raw=true" alt="grayscale" width="400">   

 - **Segmented and retouched**
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/segmentasi_hitam_putih.jpeg?raw=true" alt="segmentasi_hitam_putih" width="400">





## Action
There are three actions that the agent can do.
 - **Forward**
*Throttle* = 1; *Steer* = 0;
 - **Forward Left**
*Throttle* = 1; *Steer* = -1;
 - **Forward Right**
*Throttle* = 1; *Steer* = 1;

## Reward Function

The reward functions are different for each kinds of roundabout.

### Roundabout with intersection
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/target_lane_line.png?raw=true" alt="image" height="200">   

 1. **Angle Deviation**.
The value is `Reward1  = 1/alpha`. Alpha is the difference in angle from the target line and the agent angle.
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/reward_anglediff_sketch.jpg?raw=true" alt="image" height="260">

 2. **Distance Deviation**.
The value is `Reward2  = 1/distance_deviation*10`. Distance deviation is the distance from agent to the target line in m.
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/reward_deviasi_jarak.png?raw=true" alt="image" height="260">

 3. **Collision**.
The value is `Reward3  = -1`. Collision event triggered when the agent touched another object.

4. **Agent too far**.
The value is `Reward2  = -0.5`. Agent too far event triggered when the distance between the center of roundabout and the agent is more than 30 meters.
 
5. **Total Reward:**.
 The total reward is `Reward =  Reward1 + Reward+2 + Reward3 + Reward4`

### Roundabout without intersection
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/waypoint.png?raw=true" alt="image" height="200">

 1. **Angle Deviation**.
The value is `Reward1  = 1/alpha`. Alpha is the angle difference between the agent's direction and the direction from the agent to the nearest waypoint + 5 from the agent.
<img src="https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/reward_alpha.png?raw=true" alt="image" height="260">   

 2. **Collision**
The value is `Reward2  = -1`. Collision event triggered when the agent touched another object.   


 3. **Total Reward:**
 The total reward is `Reward =  Reward1 + Reward+2`

## Training

Using reinforcement learning with deep q network.

![methodology](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/metodologi.png?raw=true)

