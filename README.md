# EFM32_Base

This is a repository of a project done in our bachelor.

We display in Silicon Labs app on our PC, the angle and the G force, of each axis.

This project is done with ** connected to a EF32 Giant Gecko board.

The code flow pass thorugh three different tasks, connected via queues. Also, the information will be displayed only when the button 1 of the board is pressed. This is controlled by a Semaphores_ISR.
