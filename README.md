# EFM32_Base

This is a repository of a project done in our bachelor.

We display in Silicon Labs app on our PC, the angle and the G force, of each axis.

This project is done with the SparkFun LSM9DS1 accelerometer module connected to a EFM32 Giant Gecko board.

The code flow pass thorugh three different tasks, connected via queues. Also, the information will be displayed only when the button 1 of the board is pressed, controlled by interruptions and semaphores.

Made by Joaquín Borràs Sarsa and Raúl Iglesias Paredes.
