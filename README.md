This is the repository for Alisha and Marina's code for their final project for E155-Microprocessor Systems, which is building a real life system imitating Remy the Rat from Ratatouille. The system will integrate two IMUs while controlling two servo motors and communicate using I2C and RF, using an upDuino FPGA and two STM32 microcontrollers to implement this design.

The FPGA folder includes the entire Lattice project as well as the isolated top.sv file for ease of viewing core FPGA code. 

The MCU folder includes code for the two MCUs used in this project, including the Segger project and all lib and src files. 

This design uses the position estimator and filtering software Fusion from https://github.com/xioTechnologies/Fusion/tree/main, this is included in the mcu folder. 
