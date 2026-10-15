# Robin's Notebook
## 8/27/2026: Senior Project Idea Pitching
__Coming up with a final project idea__

We thought of an automatic window blind capable of automatically closing and opening using a light and temperature sensor. It will take in the room temperature and compare it to the light intensity outdoors and will decide whether the blinds should be open or closed.

We decided to add a couple of innovations, such as a mobile app implementation, so it can be customized by the user.

We also considered portability so the curtain itself will roll into the main compartment. Possible idea of easy installation using the friction method that a shower curtain pole uses.

Also considered solar panel implementation, not as the main power source, but as a secondary power source (to save power consumption).


## 9/2/2026: Senior Project Idea Pitching
__New project idea__

After consulting the professor on the automatic curtain device, the idea was abandoned because it was “too difficult to demonstrate” and had nothing innovative about it.

A new idea was made after looking at the group member’s dolly.

We will make a multi-purpose load-carrying robot capable of carrying rated loads while following the user.

The device will trail behind the user at a certain distance and will have sensors around its body to avoid obstacles such as walls or people.

The device will be controlled by a wireless remote device. The robot should be capable of traveling in diverse terrains. This includes climbing staircases; we are still attempting to settle on a design, and a couple of inspirations come from wheelchairs or RC cars that are shown climbing staircases.

Because of this feature, the robot will have a weight limit where it will alert the user when the rated load limit is exceeded.

As of now, we are attempting to get project approval first before moving forward in our thinking process.

__Reference Link:__ 

[Wheelchair going up stairs reference](https://www.youtube.com/shorts/8b46ViqHrco)

[Robotic platform with tracks going up stairs and ramps](https://www.youtube.com/shorts/r-vxdZTXooI)

[Complex robot driving on stairs](https://www.youtube.com/watch?v=9Od0E53R8PE&feature=youtu.be)

[Electric device used to help carry load up stairs](https://www.youtube.com/shorts/B11AOdH1fYY)

## 9/4/2026: TROLL E Features
__Considering options for the staircase operation__
__Writing up proposal for approval__

After debating, we considered that automatic staircase operation might be too dangerous and difficult to implement. Instead, we decided to implement a feature that can assist the user in carrying the device (pull up) and down the stairs if the user decides to carry the load through the stairs.

May not be innovative, but considering safety and course limitations, this seems like the appropriate decision.

The project proposal was written up following the template, and we are checking over it to see if there is anything to fix or add.

We also planned out meeting times and possible prototype construction to assist visualization.

*Proposal post submitted on 9/05/2026

__Manual Staircase Operation Implementation Reference:__

[Possible manual staircase implementation reference](https://www.youtube.com/watch?v=uDIoLHIY1ko)

## 9/9/2026: Project Approved
The professor has replied to our initial proposal post and asked a couple of questions regarding the design choices.

__Questions & Response:__

__What type of drive and steering system will we use?__ We will be coding the wheels to move omnidirectionally for steering. There are several online resources to achieve this. 

__How many wheels will be powered?:__ 4 total wheels will be powered with their own motors.

__Will you use mecanum wheels?:__ Yes, it will be more efficient at navigating through obstacles, and suspension is being considered to support the load.

Our project was approved upon answering those questions, and now we must consider writing up our official proposal and specific components or decisions for the robotic device.


## 9/16/2026: Writing Up Official Proposal

The official proposal for the TROLL E project was written up. We came together to make agreements on the overall direction of the project and define some of the operating parameters, such as the desired trailing distance, maximum speed it can travel, obstacle stopping distance, and payload limit.

For my part, I worked on several of the technical sections, including the drivetrain subsystem, control subsystem, power subsystem,weight sensing system, ethical, safety, and societal impact section of the proposal and organized the citations for the proposal.

I also created the main system block diagram based off of Ben's sketch to show how the different subsystems connect with each other. This helped organize the overall design and made it easier to see how power and data would interact and move through the system.

## 9/24/2026: Team Contract

The team contract was written up and signed by all group members. We established expectations for communication, participation, decision-making, meeting times, and how responsibilities would be divided throughout the project.

We agreed to communicate with each other through Discord and in-person discussions, especially when changes are made to important parts of the project such as the PCB design, major design decisions, component purchases, and changes to project are expected to be discussed as a team rather than decided by one person alone.

We also assigned general areas of focus based on our prior experience. Ben will focus mainly on sensors and the microcontroller, Daniel on the motors and battery system, and I will focus on sensors while helping with other areas when needed. We also agreed that these roles are flexible and that everyone should understand how the complete system works rather than only knowing their assigned subsystems.

I reviewed and signed the contract on 9/24/2026, agreeing to follow the team contract.

## 9/30/2026: Finishing the Design Document

The main goal for this day was to finish and refine the Design Document by finalizing more of the major design choices for TROLL E.

We continued choosing specific components for the project, including the weight sensing system and the UWB following system. We decided to use multiple load sensors with a combinator and HX711 so that the total cargo weight can be measured when the load is not perfectly centered.

We also decided to abandon the use of LiDAR and focus on ultrasonic sensors for obstacle detection. This simplified the sensing subsystem and reduced the number of sensing methods we needed to integrate.

The UWB system was also solidified so that TROLL E would use three UWB tags mounted on the cart and a user-carried anchor to estimate the user's relative position and maintain the desired traveling distance.

I have also updated the ethics, safety, and societal impact section, including concerns such as unintended movement, overload condition, battery safety, misuse, and responsibility to communicate the limitations of the device.

As a team, we also finalized the main goals that TROLL E would need to meet for the project to be considered successful. 

I also updated the original system block diagram so that it matches the newly selected components and the updated subsystem design.

## 10/1/2026: Team Design Review Sign Up

We have signed up for a team Design Review Presentation at 17:30 on Wednesday and I personally also signed up for the peer review session on the same day at 17:00.

## 10/2/2026: Finalizing Team Design Document

The team got together and reviewed the design document for errors, citations, and component choices. I made a couple of fixes on the system block diagram and reuploaded the image to the doc. After finalizing and agreeing with the document content, the Design document was submitted.

## 10/3/2026: KiCAD STM32 PCB Designing

As part of the course requirement, our team decided to make a custom PCB based around the STM32G474RET6 so that it could act as the MCU for TROLL E.

Daniel began putting together the schematic in KiCad and selected the main components needed for the board. Once the initial schematic was put together, the three of us reviewed it as a team to make sure the design was valid and that the selected components matched the needs of the project.

The goal was to catch any major issues early and make sure everyone understood how the PCB would connect the different subsystems before moving forward with the board layout.

## 10/6/2026: PCB Design Review with TA

We emailed our TA to double-check the STM32 PCB schematic before moving forward with the design. The goal was to make sure that the components we selected and the overall circuit setup was valid for TROLL E. Our TA looked over the schematic and told us that the overall design and selected parts looked good. With that confirmation, we moved forward with organizing the PCB layout and wiring the components together for the next step.

## 10/7/2026: Team Design Review

__Taking Feedback__
During our design review, a few points were brought up that we need to do more research in and see if we need or should include them in our design. Below is a list of takeaways I wrote down during the review:
* Stall current, if TROLLE missed an obstacle and is driving into it without moving forward
* If motors differ slightly, how will we ensure they all rotate at the same speed or the speed desired for omni-movement?
* Slopes, and how to close feedback loop and slow down on negative slopes
* Odometry Pods, to track distance travels, has x and y positions; this will require additional inputs into the MCU
* E-stop to cut all power (some form of autonomy, must require this)
* Low-Voltage Current protection.
We need to focus on first developing a working PCB, but will follow up on these once the design is finished.

## 10/8/2026: PCB Finalization and Ordering

I managed to finish the remaining wiring for the STM32 PCB and sent the finished PCB to Ben for final review, testing, and ordering.

During the reviewing process and after consulting with our TA, we found several problems with the PCB design. One of the major issues was that the board was larger than the course's standard PCB size allowed for ordering. This issue was also caught late during the review process, and our TA gave us an extension so we would have enough time to resize the board and resolve the problem. 

After fixing the size issue and correcting the remaining errors found during testing, the PCB passed the required checks and was approved. We were then able to submit the board during the second round of PCB ordering.

## 10/12/2026: Breadboard Demo Sign-Up and GitHub Transition

We signed up for the 11:30 breadboard demo as per course requirement.

We also decided to transition our lab notebook and project records to an electronic format using GitHub. This gave us a more organized way to keep track of our progress and made it easier for all three team members to share documents, PDFs, images, reference links, and other project files in one place.

## 10/14/26: Breaboard Demo
__Complexity of Whats Complete__, __Integrate Different Sensors__, __and Test Ultrasonic__

We need to change out our testing board to a dev board using our MCU, which Daniel is doing immediately. Additionally, we have been recommended using another sesnor for detection, or solidifying perfect execution of the ultrasonic sensors for forward sensing. Not only do we have not enough sensors, using more varied sensors adds to complexity and vision.

PCB is ordered and coming within a week, we will need to fully test everything before then so that we can actually test the PCB design.

While testing the Ultrasonic, we can see that it is somewhat accurate, sitting around 1-2 cm of error, but will easily read shorter than what we expect if something is slightly in view. This is also a test without a tape measure to confirm, so we will need some method to verify another day. We also noticed that it focuses on the closest obstacle in its field of view, so this will likely remain our safety sensor for obstacle detection as it sees anything close easily. 

I am currently looking into 2D scanning LiDAR and angle-capable mmWave radar as possible options for longer-range obstacle detection, with 2D LiDAR currently looking more promising. I will be looking further into how each option could be implemented into our current design, such as coverage, accuracy, cost, power requirements, and how it will be connected to the STM32. I will be sharing my findings with the rest of my team members before deciding how much additional obstacle detection and naviagtion capability we want to add to TROLL E.