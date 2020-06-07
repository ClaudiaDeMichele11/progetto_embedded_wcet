################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o: ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L475xx -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/vl53l0x" -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components/mx25r6435f" -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components/Common" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/Message" -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components/lis3mdl" -I../Drivers/CMSIS/Include -I../Core/Inc -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components/lsm6dsl" -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components/lps22hb" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/B-L475E-IOT01" -I"C:/Users/claud/STM32CubeIDE/workspace_1.1.0/FireSystem/Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components/hts221" -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

