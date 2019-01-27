################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FwPrConfig.c \
../Src/FwPrCore.c \
../Src/FwPrDCreate.c \
../Src/FwPrSCreate.c \
../Src/FwRtConfig.c \
../Src/FwRtCore.c \
../Src/FwSmAux.c \
../Src/FwSmConfig.c \
../Src/FwSmCore.c \
../Src/FwSmDCreate.c \
../Src/FwSmSCreate.c \
../Src/MainFsm.c \
../Src/main.c \
../Src/stm32l0xx_hal_msp.c \
../Src/stm32l0xx_it.c \
../Src/system_stm32l0xx.c 

OBJS += \
./Src/FwPrConfig.o \
./Src/FwPrCore.o \
./Src/FwPrDCreate.o \
./Src/FwPrSCreate.o \
./Src/FwRtConfig.o \
./Src/FwRtCore.o \
./Src/FwSmAux.o \
./Src/FwSmConfig.o \
./Src/FwSmCore.o \
./Src/FwSmDCreate.o \
./Src/FwSmSCreate.o \
./Src/MainFsm.o \
./Src/main.o \
./Src/stm32l0xx_hal_msp.o \
./Src/stm32l0xx_it.o \
./Src/system_stm32l0xx.o 

C_DEPS += \
./Src/FwPrConfig.d \
./Src/FwPrCore.d \
./Src/FwPrDCreate.d \
./Src/FwPrSCreate.d \
./Src/FwRtConfig.d \
./Src/FwRtCore.d \
./Src/FwSmAux.d \
./Src/FwSmConfig.d \
./Src/FwSmCore.d \
./Src/FwSmDCreate.d \
./Src/FwSmSCreate.d \
./Src/MainFsm.d \
./Src/main.d \
./Src/stm32l0xx_hal_msp.d \
./Src/stm32l0xx_it.d \
./Src/system_stm32l0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32L031xx -I"/home/achim/mnt/Electronik/Pflamiku/stm_waage/2019-01-10-pflamiku_3er_fwfsm/Inc" -I"/home/achim/mnt/Electronik/Pflamiku/stm_waage/2019-01-10-pflamiku_3er_fwfsm/Drivers/STM32L0xx_HAL_Driver/Inc" -I"/home/achim/mnt/Electronik/Pflamiku/stm_waage/2019-01-10-pflamiku_3er_fwfsm/Drivers/STM32L0xx_HAL_Driver/Inc/Legacy" -I"/home/achim/mnt/Electronik/Pflamiku/stm_waage/2019-01-10-pflamiku_3er_fwfsm/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"/home/achim/mnt/Electronik/Pflamiku/stm_waage/2019-01-10-pflamiku_3er_fwfsm/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


