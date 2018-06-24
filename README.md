# RTOS
This project indents to build a ground-up RTOS fro Tiva TM4c123gxl board. 
It includes co-operative and pre-emptive mode.
The ckt consists of 5 LEDs and 4 push buttons.

There are 8 threads - 
  Lengthyfn - Turns RED led on/off for 4 sec 
  Flash4hz - Toggles GREEN led every 125msec
  oneshot - Turns YELLOW LED on for 1 sec
  Important - Toggles BLUE led every 1 sec
  ReadKeys - Key1 - Toggles YELLOW & Turns on RED
           - Key2 - Turns on YELLOW for 1 sec & Turns off RED
           - key3 - Creates a new thread Flash4Hz
           - key4 - Destroys thread flash4Hz
  Uncooperative - Checks if button is pressed and yeilds
  Idle - Turns on/off ORANGE led
  Shell - Print PID, CPU time, create, Kill threads from terminal.
  
  It has 4 kernel functions - 
  Yeild, Wait, Sleep, POst
  
Resource management is through Semaphores- 
  1 semaphore for Lengthly and important -  Counting with 1 resource
  5 semaphore for oneshot - Counting with 5 resource
  1 semaphore for Read key Binary
  1 semaphore for keyreleased Binary
  
  Priority inheritance is taken care using priority inversion. And Scedualar is a priority based scedular.
  
  **The stack of each thread is initialized as if it has been run before.
           
          
