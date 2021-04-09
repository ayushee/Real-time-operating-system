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
           
          
Work flow -

**Stack laylout** -

Memory | Register
----------------------------
255.   | xPSR - Program Status Register 
254.   | PC - Program Counter - Keeps state of current state of the program
253.   | LR - Link Register - Stores the return information for subroutines
252.   | R12
251.   | R3
250.   | R2
249.   | R1
248.   | R0
247.   | LR ---- This is done by the compiler(It doesn't know that we are saving LR)
246.   | R3 ---- This is done by the compiler(ARM has stack alignment of 8, so it's easy)
245.   | R4
....
238    | R11

**Task control block (tcb)** -
Contains the following -
1. State - can be invalid/ready/bloacked/delayed
2. pid(process ID) - should be unique
3. Stack pointer
4. skip counter
5. priority
6. current priority
7. ticks
8. name
9. semaphore
10. startCount
11. endCount
12. totalCount
13. semaphoreUsing
Each thread has it's own tcb and there is a system tcb which contains contents of running thread.

**1. Creating thread** - 
1. Takes args - function pointer, name, priority
2. Checks if the thread has been already created (to prevent reentrancy) - If yes, reinits the tcb for the found thread
3. Else, "makes the thread stack look like it's run before" - this is done by assigning "garbage" values to regs 254 - 247 i.e. PC to LR and Reg 255 i.e xPSR is assigned with pid. The SP of the thread tcb points to reg 247 i.e LR and the priority, name and skip count are initalized

**Semaphore structure**
Contains
1. count
2. queueSize
3. ProcessQueue - stores index of tasks
4. name

**Creating Semaphore**
1. Takes args - count, name. Returns a pointer to the semaphore
2. If the semaphoreCount (a global variable) < MAX, it assigns an semphore[semaphoreCount++] to a temp ptr semaphore, assigns the count and name and returns the temp pointer semaphore
3. else returns 0

**RTOS schedular**
1. Start with 0th task (highest priority)
2. If the task state(tcb[task].state) is ready and its's skip count is greater than priority, return that task.
3. Else check for the next available high priority task

**Retriving the SP from the thread stack (putSP)**
1. Copy the contents of R0(argument to the function) to SP
2. Write the address of the next instruction to LR

**Storing the SP back onto the thread stack(getSP)**
1. Copy the value of SP to R0(function returns this value)
2. Write the address of the next instruction to LR

**Starting the RTOS**
1. Call the RTOS schedular
2. Store the Stack pointer of the task to the system task (Global variable)
3. Pop r4-r11 and pc to system task

**Yield**
1. POP r3, lr (to adjust the stack size since the compiler will push r3, lr again)
2. PUSH r4-r11 and lr to the task stack
3. getSP the task's SP to tcb - this also stores the new LR to task stack
5. call the rtos scedular
6. putSP the new task to system task
7. pop r4-r11 and pc
