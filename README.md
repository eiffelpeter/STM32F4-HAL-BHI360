
## Install STM32 tools
[STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)  
[STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)  
[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)  

## Clone code
`git clone https://github.com/eiffelpeter/STM32F4-HAL-BHI360.git`  


## Connect STM32F4-Discovery and BHI360 via I2C
![IMAGE ALT TEXT HERE](./Doc/IMG_8921.jpg)  

## I2C interface connections for BHI360 shuttle board
![IMAGE ALT TEXT HERE](./Doc/bhi360_shuttle_board.png)  

![IMAGE ALT TEXT HERE](./Doc/bhi360_shuttle_board_2.jpg)  


## Build and Download
1. Open `STM32F4-HAL-BHI360.ioc` file in the STM32CubeMX and generate code.  
2. Open STM32CubeIDE and import by `Import Atollic TrueSTUDIO Project`.  
3. Open STM32CubeProgrammer and select `STM32F4-HAL-BHI360.hex` to Download.  
4. After Download, press STM32F4 reset button to run.  
5. LD4 LED blink when read sensors successfully.
6. Console [log](./Doc/log.txt) print tx at STM32 PA2 ( baud rate 115200 ).  
![IMAGE ALT TEXT HERE](./Doc/console.png)  


## Check build log for flash and ram size
```
   text	   data	    bss	    dec	    hex	filename
 183076	    468	  28072	 211616	  33aa0	STM32F4-HAL-BHI360.elf
```


## Build error
Please check STM32Cube path in .cproject and .project files.  
![IMAGE ALT TEXT HERE](./Doc/stm32cube_path.png)  


## Reference
[STM32-HAL-BHI360](https://github.com/Dmivaka/STM32-HAL-BHI360/tree/main). This [document](./Doc/howto_build_STM32G4.docx) show howto build STM32G4.  
