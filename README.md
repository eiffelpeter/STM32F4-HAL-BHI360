
## Install STM32 tools 
[STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)  
[STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)   
[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)    

## Clone code
`git clone https://github.com/eiffelpeter/STM32F4-HAL-BHI360.git`

## Connect STM32F4-Discovery and BHI360
![IMAGE ALT TEXT HERE](./Doc/IMG_8921.jpg)

## Pin connection for BHI360 shuttle board
![IMAGE ALT TEXT HERE](./Doc/bhi360_shttle_board.png)

## Console log 
printf tx at STM32 PA2 ( baud rate 115200 )  
[log](./Doc/log.txt)  

## Build and Download
Open `STM32F4-HAL-BHI360.ioc` file in the STM32CubeMX and generate code.  
Open STM32CubeIDE and import by `Import Atollic TrueSTUDIO Project`.  
Open STM32CubeProgrammer and select `STM32F4-HAL-BHI360.hex` to Download.  
After Download, press STM32F4 reset button to run.  

## Reference
[STM32-HAL-BHI360](https://github.com/Dmivaka/STM32-HAL-BHI360/tree/main).  
[Here](./Doc/howto_build_STM32G4.docx) is document to build STM32G4.  
