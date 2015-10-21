################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../CG3002RTOS.cpp \
../heap_1.c \
../L3G.cpp \
../list.c \
../LSM303.cpp \
../NewPing.cpp \
../port.c \
../queue.c \
../tasks.c \
../twi.c \
../Wire.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
CG3002RTOS.o \
heap_1.o \
L3G.o \
list.o \
LSM303.o \
NewPing.o \
port.o \
queue.o \
tasks.o \
twi.o \
Wire.o

OBJS_AS_ARGS +=  \
CG3002RTOS.o \
heap_1.o \
L3G.o \
list.o \
LSM303.o \
NewPing.o \
port.o \
queue.o \
tasks.o \
twi.o \
Wire.o

C_DEPS +=  \
CG3002RTOS.d \
heap_1.d \
L3G.d \
list.d \
LSM303.d \
NewPing.d \
port.d \
queue.d \
tasks.d \
twi.d \
Wire.d

C_DEPS_AS_ARGS +=  \
CG3002RTOS.d \
heap_1.d \
L3G.d \
list.d \
LSM303.d \
NewPing.d \
port.d \
queue.d \
tasks.d \
twi.d \
Wire.d

OUTPUT_FILE_PATH +=CG3002RTOS.elf

OUTPUT_FILE_PATH_AS_ARGS +=CG3002RTOS.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+=  \
C:/Users/caiyunyang/Desktop/CG3002/Arduino/Arduino/Debug/libArduino.a

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler























./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DNDEBUG  -I"../../../arduino-1.0.6/hardware/arduino/cores/arduino" -I"../../../arduino-1.0.6/hardware/arduino/variants/mega" -I"../../../FreeRTOSV8.1.2/FreeRTOS/Demo/Common/include" -I"../../../FreeRTOSV8.1.2/FreeRTOS/Source/include" -I"../../../FreeRTOSV8.1.2/FreeRTOS/Source/portable/GCC/ATMega2560"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -mmcu=atmega2560 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

./%.o: .././%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DNDEBUG  -I"../../../FreeRTOSV8.1.2/FreeRTOS/Source/portable/GCC/ATMega2560" -I"../../../arduino-1.0.6/hardware/arduino/cores/arduino" -I"../../../arduino-1.0.6/hardware/arduino/variants/mega" -I"../../../FreeRTOSV8.1.2/FreeRTOS/Demo/Common/include" -I"../../../FreeRTOSV8.1.2/FreeRTOS/Source/include"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -mmcu=atmega2560 -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Linker : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="CG3002RTOS.map" -Wl,--start-group -Wl,-lm -Wl,-lArduino  -Wl,--end-group -Wl,-L"C:\Users\caiyunyang\Desktop\CG3002\Arduino\Arduino\Debug"  -Wl,--gc-sections -mrelax -mmcu=atmega2560  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "CG3002RTOS.elf" "CG3002RTOS.hex"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "CG3002RTOS.elf" "CG3002RTOS.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "CG3002RTOS.elf" > "CG3002RTOS.lss"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "CG3002RTOS.elf" "CG3002RTOS.srec"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-size.exe" "CG3002RTOS.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "CG3002RTOS.elf" "CG3002RTOS.a" "CG3002RTOS.hex" "CG3002RTOS.lss" "CG3002RTOS.eep" "CG3002RTOS.map" "CG3002RTOS.srec" "CG3002RTOS.usersignatures"
	