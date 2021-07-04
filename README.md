# # Carla RL-DQN Thesis Project

A bachelor thesis project about **autonomous car maneuver** around **roundabout** using **reinforcement learning** with **deep q network**

## Environment Setup

Two kinds of roundabouts are used in this research.
 - **Roundabout with Intersection**
![roundabout](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/bundaran.png?raw=true =400x)
 - **Roundabout without Intersection**
![roundabout2](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/bundaran_tanpa_simpang.png?raw=true =400x)

## Sensor
The camera sensor was obtained, and then we did the semantic segmentation using CARLA's built-in function.
![roundabout](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/rgb.png?raw=true =x100) -> ![roundabout](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/segmentasi.png?raw=true =x100)
>RGB -> Semantic Segmentation

Two kinds of camera setup are used in this research.


 - **Segmented and Grayscaled**
![roundabout](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/grayscale.png?raw=true =400x)
 - **Segmented and retouched**
![roundabout2](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/segmentasi_hitam_putih.jpeg?raw=true =400x)





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

![roundabout2](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/target_lane_line.png?raw=true =x200)
 1. **Angle Deviation**
The value is `Reward1  = 1/alpha`. Alpha is the difference in angle from the target line and the agent angle.
![roundabout2](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/reward_anglediff_sketch.jpg?raw=true =x260)

 2. **Distance Deviation**
The value is `Reward2  = 1/distance_deviation*10`. Distance deviation is the distance from agent to the target line in m.
![roundabout2](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/reward_deviasi_jarak.png?raw=true =x250) 
3. **Collision**
The value is `Reward3  = -1`. Collision event triggered when the agent touched another object.
 4. **Agent too far**
The value is `Reward2  = -0.5`. Agent too far event triggered when the distance between the center of roundabout and the agent is more than 30 meters.
 5. **Total Reward:**
 The total reward is `Reward =  Reward1 + Reward+2 + Reward3 + Reward4`

### Roundabout without intersection
![roundabout2](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/waypoint.png?raw=true =x200)

 1. **Angle Deviation**
The value is `Reward1  = 1/alpha`. Alpha is the angle difference between the agent's direction and the direction from the agent to the nearest waypoint + 5 from the agent.
![roundabout2](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/reward_alpha.png?raw=true =x250) 
 2. **Collision**
The value is `Reward2  = -1`. Collision event triggered when the agent touched another object.
 3. **Total Reward:**
 The total reward is `Reward =  Reward1 + Reward+2`

## Training

Using reinforcement learning with deep q network.

![methodology](https://github.com/roychanmeliaz/bachelor-thesis-paper-latex/blob/main/images/metodologi.png?raw=true)





