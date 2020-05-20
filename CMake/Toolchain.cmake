message("Toolchain.cmake running.")

# Target operating system
message("Setting : CMAKE_SYSTEM_NAME/VERSION/PROCESSOR")

#
set(CMAKE_SYSTEM_NAME		Linux)
set(CMAKE_SYSTEM_VERSION	1)
set(CMAKE_SYSTEM_PROCESSOR	arm)

# Compilers to use for ASM, C and C++
message("Setting : Toolchain")
set(CMAKE_C_COMPILER		arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER		arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER		arm-none-eabi-g++)
set(CMAKE_OBJCOPY			arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP			arm-none-eabi-objdump)
set(CMAKE_SIZE			    arm-none-eabi-size)

# Initialse these options to blank (thats what happens if an unknown build type is selected)
set(MSG_LEN 0)
set(OPTIMISATION "-O0")
set(DEBUG "")
set(ADDITIONAL_FLAGS "")
set(IS_RELEASE TRUE)

# This selects the compiler flags, feel free to change these :)
message("Build mode : " ${CMAKE_BUILD_TYPE})
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    message("Setting up Release flags.")
    set(OPTIMISATION	    "-O3")
    set(ADDITIONAL_FLAGS    "-flto -fomit-frame-pointer -Wl,--strip-all -fdevirtualize-speculatively")

elseif (CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    message("Setting up MinSizeRel flags.")
    set(OPTIMISATION	    "-Os")
    set(ADDITIONAL_FLAGS    "-flto -fomit-frame-pointer -Wl,--strip-all -fdevirtualize-speculatively")

elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Setting up Debug flags.")
    set(OPTIMISATION	    "-O0")
    set(DEBUG               "-g3  ")

elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    message("Setting up RelWithDebInfo flags.")
    set(OPTIMISATION	    "-O3")
    set(DEBUG               "-g3 -ggdb")

elseif (CMAKE_BUILD_TYPE STREQUAL "MinSizeRelWithDebInfo")
    message("Setting up MinSizeRelWithDebInfo flags.")
    set(OPTIMISATION	    "-Os")
    set(DEBUG               "-g3 -ggdb -ggdb3 -DDEBUG ")

else()
    message("WARNING - Unknown build configuration.")
endif()

#
set(COMMON_FLAGS "-mcpu=${CORE} --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -${ARM_ASM} ")
set(COMPILER_COMMON_FLAGS "-ffunction-sections -fdata-sections -fstack-usage -Wall -MMD -MP ") 
set(C_FLAGS "${COMPILER_COMMON_FLAGS} ")
set(CPP_FLAGS "${COMPILER_COMMON_FLAGS} -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit ")

# 
set(CMAKE_ASM_FLAGS	        "${COMMON_FLAGS} -x assembler-with-cpp    " CACHE INTERNAL "asm compiler flags")
set(CMAKE_C_FLAGS           "${COMMON_FLAGS} ${DEBUG} ${OPTIMISATION} ${C_FLAGS} ${C_PREPROCESSOR} " CACHE INTERNAL "c compiler flags")
set(CMAKE_CXX_FLAGS	        "${COMMON_FLAGS} ${DEBUG} ${OPTIMISATION} ${CPP_FLAGS} ${CXX_PREPROCESSOR}" CACHE INTERNAL "cpp compiler flags")

# You may have a linker issue. If that occurs change nano.specs to nosys.specs
set(CMAKE_EXE_LINKER_FLAGS  " -Wl,--gc-sections -static -u _printf_float  -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group" CACHE INTERNAL "exe link flags")

#-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -specs=nano.specs -TSTM32F767ZITx_FLASH.ld  -lc -lm -lnosys  -Wl,-Map=build/stert.map,--cref -Wl,--gc-sections 
#
message(${CMAKE_C_FLAGS})
message(${CMAKE_CXX_FLAGS})
message(${CMAKE_EXE_LINKER_FLAGS})

# 
message("CMAKE_C_FLAGS = "          ${CMAKE_C_FLAGS})
message("CMAKE_CXX_FLAGS = "        ${CMAKE_CXX_FLAGS})
message("CMAKE_EXE_LINKER_FLAGS = " ${CMAKE_EXE_LINKER_FLAGS})
#
# 
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
