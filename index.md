---
layout: single
author_profile: false
title: Robot Teleoperation Performance
toc: true
toc_sticky: true
mermaid: true
math: mathjax
---


# Overview

Over the past year, I have been attempting to gain insight into why robotic teleoperation is such a hard task. During this investigation, I have built demonstrations and answered questions about control precision, control kinematics, runtime performance conditions and visibility. This report is intended to serve as a summary of what I learned, my thought process throughout, and tools that I built so that future research can leverage my work.


# Initial Motivation

By working with the [Franka Emika Panda](https://frankaemika.github.io/docs/overview.html) robot, I had gained the intuition that robotic teleoperation was not a solved problem. The question of how to control a robot arm at a low level is well studied{% cite kent2017comparison %}; there are several different well established paradigms for providing inputs with a variety of control schemes, handling haptic feedback, and providing feedback to the user. More recently, novice users were able to control a robot arm with off-the-shelf commerical hardware {% cite rakita2017motion rakita2017methods %}. This method of "Mimicry" based control allowed users to perform tasks with higher dexterity than previuously thought possible (e.g. perform TinkerToy[^1] style assembly). However, the actual performance of human + robot systems were not compelling. I had seen (and have yet to see) a teleoperation system that came close to human performance. While this did not remove the value of such systems, because there were so many variables, it was not clear what was causing the performance drop.

# Initial Review

## Existing Research

Over the past few decades, there have been a number of papers that explore teleoperation limitations. However, these are generally focused on low level factors, like latency {% cite lum2009teleoperation, kaber2000effects %} or the limits of haptic feedback {% cite salle2001analysis chaudhari2011opening %}, or analyzing the human factors {% cite lathan2002effects %}. The research that does holistically evaluate the system performance in terms of both input and feedback systems focus on small model tasks and don't propose a method to achieve human level performance {% cite chen2007human %}. 

The concensus is that the all of the following factors impact performance, but it is not clear which one(s) is(are) the bottleneck(s) for a certain task:

1. Visual Feedback
   1. Field of View
   2. Presence of Multiple View Points
   3. Ego/Ergo-centric Perspective
   4. Depth Perception
   5. Motion of POV
2. Control System
   1. Orientation of Controls WRT Robot
   2. Time Delays


From this review, and review of recent teleoperation applications, it was clear that there was a performance gap and it was not obvious where it comes from in robotic arm teleoperation applications{% cite rea2022still %}. 


## Initial Hypotheses

To explore the limitations myself and build an intuition for the difficulties, I set up an initial comparison between myself teleoperating the Panda with our in-lab developed real time control system and an HTC Vive controller, and a fully automated, open-loop behavior. 

| Teleoperated | Repeating Demonstration |
| :---: | :---: |
| ![Hand Sealant]({{site.baseurl}}/assets/imgs/2022-05-02-13-50-01.png) | {::nomarkdown}<iframe src="https://drive.google.com/file/d/1GLRXsyCzztEiTp7Q-7AdvauyvyqjnAB4/preview" width="640" height="480" allow="autoplay"></iframe>{:/} |

This task was chosen because it contained time pressure and required precise trajectory control, but did not contain force interaction. The result was clear that our current teleoperation system was not sufficient. At this time, I was able to identify several potential points of difficulty.

### Control Parameters

What is most apparent from the video is that it was not possible to either elicit the necessary high frequency motion of the end-effector as well as react to positioning errors. This was due to two factors: a low-pass filter in the realtime controller, and a latency between user input and robot end-effector. The low-pass filter is typically necessary in real-time control applications to filter out input signal noise and to prevent commanding dangerous motions from the robot. Potential solutions to this issues are discussed in [Solution Development](#heading-solution-development). The presence of latency is a known issue and has been explicitly explored in a number of teleoperation contexts{% cite lum2009teleoperation rakita2020effects %}. One succinct study demonstrated that any latency above 75ms had a measurable impact on the motor feedback loop{% cite waltemate2016impact %}.

To analyze the latency between the human hand position and robot hand position, the following system was measured.

<div class="mermaid">
graph LR;
  HumanHand -->|0ms| Vive;
  Vive -->|x ms| Computer;
  Computer -->|y ms| RobotController;
  RobotController -->|z ms| RobotHand;
  RobotHand -->|0ms| Vive2;
  Vive2 -->|x ms| Computer;
</div>

We seek the sum $$ latency = x + y + z $$. By attaching a Vive controller to the robot hand, the human can be removed from the equation etirely, and we only need to measure the round trip time delay between sending a command and getting one back through the Vive controller. By commanding the robot to follow a circular trajectory, $$ \vec{r}(t) = (0.4 + 0.1cos(t))\hat{x} + 0.1sin(t) \hat{y} + 0.5\hat{z} $$, we can look at the displacement from the initial position as a function of time for both the commands send and the resulting position recorded. Then, finding the optimial shift time that maximally aligns the curve yields the following result.

| --- | --- |
| ![Latency Motion]({{site.baseurl}}/assets/imgs/2022-05-03-13-15-31.png) | ![]({{site.baseurl}}/assets/imgs/2022-05-03-13-23-25.png) |


This suggests the system has a total latency of approximately 220ms. However, upon closer inspection, this turned out to be a combination of *true latency*, which is the amount of time for information to pass through the system as well as *dynamic latency*, which delay caused by physical limits placed on the robot's motion. As discussed by Rakita et al. {% cite rakita2020effects %}, these types of latency impact user performance differently and should be considered separately. After looking more closely at the amount of time between a command sent and motion detected, the *true latency* of the system was measured at approximately 50ms. This is consistent with the reported latency of the HTC Vive controllers (20ms) in addition to network latency and other processing steps that are required. These results suggest that the communication latency is not likely a significant source of error in this system, but it is possible that the dynamic latency is reducing performance.


### Visibility

Despite being co-located with the robot arm, I found that visibility was already an issue. Simply being displaced a few feet away from the task significantly reduced my ability to judge the precise position of the end effector. This strongly suggested that teleoperation performance is highly dependent on specific qualities of the visual feedback. To further explore this, I had a lab member complete a small comparison. They were asked to perform the sealant task twice. Both times they completed it without robot involvement. The first time was with no modification. The second time, however, was while wearing an HTC Vive headset with visual pass-through enabled. This modified their vision from stereoscopic to monoscopic, and reduced the fidelity of the information by reducing the resolution in spatial and color dimensions. While they performed the task relatively easily in the first case, they were unable to precisely position the tool in the second condition and resorted to tapping the tip on against the part to align themselves.


### Input Device Kinematics


One additional observation was that the kinematics of the sealant gun were significantly different than the kinematics of the controller. 

| Input Device | Sealant Gun |
| :---: | :---: |
| {::nomarkdown}<iframe src="https://drive.google.com/file/d/1GLRXsyCzztEiTp7Q-7AdvauyvyqjnAB4/preview" width="448" height="336" allow="autoplay"></iframe>{:/} | {::nomarkdown}<iframe src="https://drive.google.com/file/d/1G7Je9HiTtSpk-5DH0KAve5c613w6PjbO/preview" width="448" height="336" allow="autoplay"></iframe>{:/} |


In particular, because the sealant gun is used with one hand acting as a fulcrum, the user can be much more precise with the angle of the tool while using both hands to stabilize the position. To determine if this was a significant factor, the offset orientation was modified and the Vive controller was placed inside of the gun. The subjective consensus from subsequent tests was that the new kinematics resulted in a much more natural and comfortable input device for the task, but did not remove the fundamental performance bottleneck.


# Solution Development

To safely control the Panda robot, there was a conservative low-pass filter in place, which prevented commands from resulting in high jerk, accelleration, or velocity. This was done to keep the system within the safe range that is outlined by [Franka Emika](https://frankaemika.github.io/docs/control_parameters.html#necessary-conditions). This is not a perfect solution, however, since such a filter cannot in general prevent violation of these limits unless there are assumed constraints on the range of the commands. Similarly, it has to be extremely conservative to avoid all of the limits in practice. Because of this, I explored multiple techniques for generating time optimal trajectories, with moderate success.

## Constrained Optimization for Inverse Kinematics

When a cartesian command comes from the user input source, it needs to be converted into a joint command to control the robot arm. Since all of the relevant dynamic constraints are specified in joint-space, this seemed to be a natural place to enforce those constraints are met. 

This was done by converting all of the higher order constraints into joint-position constraints based on the current robot state and using a [Forward Euler](https://en.wikipedia.org/wiki/Euler_method) discrete differentiation scheme. This is the same scheme that the Franka Emika driver software uses to check if your commands are obeying the limits. The following linear inequalities outline that constraint conversion,with the lower and upper bounds represented by $$ \_^-~\text{and}~\_^+ $$, respectively.

$$
\begin{aligned}
  q^- &\leq ~&q^{t+1} &\leq q^+ \rightarrow & q^- &\leq &q^{t+1} &\leq q^+ \\
  \dot{q}^- &\leq ~&\dot{q}^{t+1} &\leq \dot{q}^+ \rightarrow & \Delta t\dot{q}^- &\leq ~&q^{t+1}-q^{t} &\leq \Delta t\dot{q}^+ \\
  \ddot{q}^- &\leq ~&\ddot{q}^{t+1} &\leq \ddot{q}^+ \rightarrow & {\Delta t}^2 \ddot{q}^- &\leq ~&q^{t+1} - q^{t} - \Delta t\dot{q}^t &\leq \Delta t^2\ddot{q}^+ \\
  \dddot{q}^- &\leq ~&\dddot{q}^{t+1} &\leq \dddot{q}^+\rightarrow & {\Delta t}^3 \dddot{q}^- &\leq ~&q^{t+1} - q^{t} - \Delta t\dot{q}^t - {\Delta t}^2 \ddot{q}^t &\leq {\Delta t}^3 \dddot{q}^+\\
\end{aligned}
$$

Then, the typical optimization that is done to solve the IK problem was augmented to contain these additional dynamic constraints. However, I discovered that this does not often result in apprporiate behavior. Consider that if there is a target angle $$ q^* $$ which minimizes the IK cost function, the non-linear optimizer will pick the next value of $$ q $$ to be the closest to $$ q^* $$. However, this is done in a greedy way that only enforces the constraints at the current timestep. This can result in a dynamic state ($$ \{q, \dot{q}, \ddot{q}, \dddot{q}\} $$) that instantaneously satisfies the constraints, but will generate inconsistent constraints some time in the future. As an explicit example, if the joint has a state $$ \{q=q^+, \dot{q}=\dot{q}^+, \ddot{q}=\ddot{q}^+, \dddot{q}\} $$, then it is impossible to avoid violating the constraints in the next timestep. The widget below gives a visual explanation of the issue. The control system attempts to reach $$ q = 1 $$ as quickly as possible, while maintaining dynamic constraints. If the constraints are inconsistent at any point, the curve will be cut off. Try setting the bounds to $$ 2, 1, 4, 1000 $$ to see what the trajectory might look like. From there, increasing the acceleration limit will eventually cause it to become unstable.

{% include control_widget.html %}

## Time Optimal Trajectory Generation

An alternative method, that is much more effective in practice, is to separate *where* the joint is going (human input + IK) from *how* it gets there. Because the constraints bound a fourth degree differential equation, it is possible to generate a piecewise fourth degree polynomial that satisfies them. While cumbersome, this can be done analytically and was done by Berscheid and Kröger and released as the Ruckig softwar{% cite berscheid2021jerk %}. 

| ![Ruckig Plan]({{site.baseurl}}/assets/imgs/2022-05-04-08-55-46.png) |
| --- |
| Example time optimal trajectory generated by Ruckig. In this example, a trajectory is generated to take a joint from the state $$ \{-0.5, 0, 0.5, 0\} $$ to the state $$ \{0.75, 0.75, 0.3, 0\} $$. Figure taken from the [Ruckig Github](https://github.com/pantor/ruckig) page. |

Simply integrating this into the existing robot arm controller was straightforward. However, there are some issues when commanding very small positional displacements. Because Ruckig generates a trajectory that achieves the precise commanded state as quickly as possible, it gives no consideration for how long it takes to get near the state. This feature becomes an issue in the case where an approximate state can be achieved much more quickly with much less force than the exact state. As an example, moving from the state $$ \{0, 0, 0, 0\} $$ to $$ \{10^{-4}, 0, 0, 0\} $$ might not actually require moving at all from a user's perspective since the system is not likely to be that precise in the first place. However, generating a trajectory plan from Ruckig with realistic dynamic constraints might result in a motion plan with a duration of ~150ms. While this time seems relatively small, during the trajectory significant jerk and acceleration were achieved. As a result, consistently generating such a motion plan is not appropriate for noisy real-time control systems, without additional filtering.


# Virtual Investigation

After exploring the possibility of optimizing the real-time control of the physical Panda robot, I determined that it was much more feasible to investigate the performance factors of robot arm teleoperation systems in simulation than in reality. By working in a virtual environment, it is possible to explore the impact of different factors over a much larger parameter space than is currently physically possible. For example, it is very difficult to create a real environment where the latency is below that of our current system, but perfoming the task virtually could give us insight into whether or not it is valuable to pursue a lower latency system.

## Virtual Sealant

When designing the virtual environment, I initially aimed to replicate the real sealant task with the following adjustable variables:

1. Maximum cartesian velocity
2. Low-pass filter frequency
3. Distance to workspace
4. Fixed vs Dynamic view point


| {::nomarkdown}<iframe src="https://drive.google.com/file/d/1WQ_zmJvkCRyasmeRxCLONaS1OpWZNsuq/preview" width="640" height="480" allow="autoplay"></iframe>{:/} |
| --- |
| Virtual Sealant task simulated in a Unity environment. |

This task provided useful insight into performance factors, but was difficult to objectively evaluate and ultimately not very representative of the class of tasks that could be done with a teleoperated system.


[^1]: [Tinker Toys](https://en.wikipedia.org/wiki/Tinkertoy)




{% bibliography %}