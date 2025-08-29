# Application 1 Analysis
### Question 1 Vary Priorities: 

When the print task and blink task are both at a priority of 1 the blink task will always print first when both tasks are at time 10010ms which causes the print task to print at 10020ms and the first period to be 10010. After the first period the print task is consistently at a period of 10000. When the print task is set to priorty 2, the first print task will print at 10010ms with a period of 10000 which stays the same for each period. The blink task also prints at 10010ms even though it prints second. 

If the delay is set higher in the print task the LED will not change since they are independent events

If the number of characters printed was so high that it caused the CPU to buffer while printing the characters then the LED may not blink on schedule because the CPU is being used to print while the lower priority LED is waiting for access to the CPU. 

### Question 2 Increase Load:
When I got rid of the delay the LED did not blink or print to the console due to the CPU load required from the print tasks

### Question 3 Thematic Customization:
As a developer using less word to describe a task will save CPU computation making the program more predictable. Being on time and correct are the heart of real-time systems and for a spacecraft specifically if the light stopped blinking or was blinking irregularly it would indicate an issue with the spacecraft. Detecting this early would give engineers the oppurtunity to fix the issue before a serious failure occurs. The same is true for printing the status message. In this system, predictability and correctness is needed to monitor the spacecrafts health and to address issues before they escelate. 

## Question 4 Identify Period:
a. I measured the periods by first acquiring the current time of the system. I then created a variable to hold the previous time from the previous runtime and initialized it to 0. Inside the loop I first update the previous time to be the value of current time which holds the time calculated during the previous loop execution. I then update the current time and subtract the current time from previous time to get the period. 

b. The period for LED blink task is 250ms

c. The period for the print task is 10000ms

## Question 5 Timing Requirements:
My system met the timing requirements because I printed the times when each task executed using the code 
TickType_t currentTime = pdTICKS_TO_MS(xTaskGetTickCount()); and confirmed that the blinks were happening every 250ms and that the print task happened every 10000ms (10s). 

## Question 6 Single-Loop Timing:
In the single-loop code the LED does not meet its timing requirement because of the delay caused by the print task. Since the print and LED task are dependent on eachother the delays effect each task and cause the system to not work as intended.

## Question 7 Timing Determinism:
I disagree. Although this project worked using multitasking, if the CPU had to run too many tasks then the application would not be deterministic as highlighted when we flooded the application with print messages. This caused the LED to stop blinking which makes the application not deterministic. 