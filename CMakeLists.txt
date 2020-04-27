cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

# Name your target information here (Should be the same name as detected by your HAL library #defines)
set(TARGET          STM32F767xx)
set(ARCH			armv7ve)
set(CORE			cortex-m7)
set(ARM_ASM			mthumb)

# Configure the C and C++ standards
SET(CMAKE_C_STANDARD 11)
SET(CMAKE_CXX_STANDARD 17)
SET(C_PREPROCESSOR " -DUSE_HAL_DRIVER -DSTM32F767xx -DDEBUG -DSTM32GENERIC -DARM_MATH_CM7 -D__CM7_REV=0x0100U -D__MPU_PRESENT=1 -D__FPU_PRESENT=1 -D__ICACHE_PRESENT=1 -D__DCACHE_PRESENT=1 ")
SET(CXX_PREPROCESSOR " -DUSE_HAL_DRIVER -DSTM32F767xx -DDEBUG -DSTM32GENERIC -DARM_MATH_CM7 ")
 
# TODO: You may need to configure this.
include(CMake/Toolchain.cmake)

# TODO: Replace ProjectTemplate with your project name.
# TODO: Replace 1.0.0 with your project version.
# TODO: Replace "Description" with your project description.
# Name your project here
project(3DPrint C CXX ASM)

#enable_language(C)
#enable_language(CXX)
#enable_language(ASM)

#
# Select from:
# - STATIC_LIBRARY
# - EXECUTABLE
set(TYPE EXECUTABLE)

# Project mode, precompiled .lib file or .elf file?

# Add your project #defines here
# These are included as #defines in the project
add_compile_definitions( ${TARGET} USE_STDPERIPH_DRIVER HSE_VALUE=48000000 )

#
# TODO: Add any additional include directories here
include_directories(
	# "Core/Inc"
    # "Drivers/CMSIS/Core/Include"
    # "Drivers/CMSIS/Core_A/Include"
    # "Drivers/CMSIS/Device/ST/STM32F7xx/Include" 
    # "Drivers/CMSIS/Include"
    # "Drivers/STM32F7xx_HAL_Driver/Inc"


    "Drivers/CMSIS/Include"
    "Drivers/CMSIS/Device/ST/STM32F7xx/Include"
    "Core/Inc"
    "Drivers/STM32F7xx_HAL_Driver/Inc"
    "Drivers/STM32F7xx_HAL_Driver/Inc/Legacy"
    "Core/Src/Marlin/src/HAL/HAL_STM32_F767xx/include"
    "Core/Src/Marlin"
    "Core/Src/u8glib/csrc"
    "Core/Src/u8glib/cppsrc"
    "Core/Src/u8glib/nonarduino"
    "Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc"
    "Middlewares/ST/STM32_USB_Host_Library/Core/Inc"
    "USB_HOST/App"
    "USB_HOST/Target"
    "Drivers/CMSIS/DSP/Include"
    "Core/Src/TMCDriver"
    "Core/Inc/HAL"
)

# TODO: Add your project source files or their containing folders here
# Setup project sources
file(GLOB_RECURSE 
    LIBRARY_SOURCE
    "Drivers/CMSIS/Core_A/"
    "Drivers/STM32F7xx_HAL_Driver/Src/*.c"
)
file(GLOB_RECURSE 
	 MAIN_SOURCE
     ${LIBRARY_SOURCE}
	"Core/src/*.c"
    "Core/src/*.cpp"
    "Core/Src/Marlinsrc/*.cpp"
    "Drivers/CMSIS/Core_A/"
    "Drivers/STM32F7xx_HAL_Driver/Src/*.c"
    "USB_HOST/*.c"
    "Middlewares/ST/STM32_USB_Host_Library/*.c"
    "startup/startup_stm32f767xx.s"
)

# TODO: Add your project linker scripts here (if you have any)
# Add all linker scripts to the linker command
file(GLOB_RECURSE
	LK_SCRIPTS
	"*.ld"
)
#

# This only does something if .ld files exist in the scripts directory
set(LINKER_COMMON " -Wl,-Map=\"${PROJECT_NAME}.map\" ")
#
#
foreach(file ${LK_SCRIPTS})
	set(LINKER_COMMON "${LINKER_COMMON} -T\"${file}\"")
endforeach()
#
#
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}${LINKER_COMMON}" CACHE INTERNAL "exe link flags")
message("CMAKE_EXE_LINKER_FLAGS = " ${CMAKE_EXE_LINKER_FLAGS})
#
#
if (TYPE STREQUAL "STATIC_LIBRARY")
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
    #
    # TODO: If the project should be build as a static library uncomment this
    add_library( build STATIC ${MAIN_SOURCE} )
else()
    # 
    add_executable(${PROJECT_NAME}.elf ${MAIN_SOURCE})
    #
    # Print the size of the .hex
    add_custom_target(size ALL arm-none-eabi-size ${PROJECT_NAME}.elf DEPENDS ${PROJECT_NAME}.elf)
    add_custom_target(${PROJECT_NAME}.bin ALL DEPENDS ${PROJECT_NAME}.elf COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin)
    add_custom_target(${PROJECT_NAME}.hex ALL DEPENDS ${PROJECT_NAME}.elf COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex)
endif()