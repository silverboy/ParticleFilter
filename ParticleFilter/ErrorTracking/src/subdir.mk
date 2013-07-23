################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Detector.cpp \
../src/FiltroParticulas.cpp \
../src/evaluarPF.cpp \
../src/svm.cpp 

OBJS += \
./src/Detector.o \
./src/FiltroParticulas.o \
./src/evaluarPF.o \
./src/svm.o 

CPP_DEPS += \
./src/Detector.d \
./src/FiltroParticulas.d \
./src/evaluarPF.d \
./src/svm.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/mrpt/base/include/ -I/usr/local/include/mrpt/mrpt-config/ -I/usr/local/include/mrpt/bayes/include/ -I/usr/local/include/mrpt/gui/include/ -I/usr/local/include/mrpt/opengl/include/ -I/usr/local/include/mrpt/maps/include/ -I/usr/local/include/mrpt/obs/include/ -I/usr/local/include/mrpt/slam/include/ -I/usr/local/include/mrpt/graphs/include/ -I/usr/local/include/mrpt/vision/include/ -I/usr/local/include/mrpt/scanmatching/include/ -I../include -O0 -g3 -Wall -c -fmessage-length=0 -fpermissive -Wno-enum-compare -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


