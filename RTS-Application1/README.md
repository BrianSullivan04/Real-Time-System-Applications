# Application 1 Analysis
### Question 1 Vary Priorities: 

When the print task and blink task are both at a priority of 1 the blink task will always print first when both tasks are at time 10010ms which causes the print task to print at 10020ms and the first period to be 10010. After the first period the print task is consistently at a period of 10000. When the print task is set to priorty 2, the first print task will print at 10010ms with a period of 10000 which stays the same for each period. The blink task also prints at 10010ms even though it prints second. 

If the delay is set higher in the print task the LED will not change since they are independent events

If the number of characters printed was so high that it caused the CPU to buffer while printing the characters then the LED may not blink on schedule because the CPU is being used to print while the lower priority LED is waiting for access to the CPU. 

### Question 2 Increase Load:
