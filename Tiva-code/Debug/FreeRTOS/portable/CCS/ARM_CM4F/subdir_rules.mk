################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/portable/CCS/ARM_CM4F/%.obj: ../FreeRTOS/portable/CCS/ARM_CM4F/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --include_path="D:/RTES/EX 5/Nihal_T_Vishal_Raj_RTES_Ex_5/Nihal_T_Vishal_Raj_RTES_Ex_5/Code/FreeRTOS/Question 2&3/FreeRTOS/portable/CCS/ARM_CM4F" --include_path="D:/RTES/EX 5/Nihal_T_Vishal_Raj_RTES_Ex_5/Nihal_T_Vishal_Raj_RTES_Ex_5/Code/FreeRTOS/Question 2&3" --include_path="D:/RTES/EX 5/Nihal_T_Vishal_Raj_RTES_Ex_5/Nihal_T_Vishal_Raj_RTES_Ex_5/Code/FreeRTOS/Question 2&3/driverlib" --include_path="D:/RTES/EX 5/Nihal_T_Vishal_Raj_RTES_Ex_5/Nihal_T_Vishal_Raj_RTES_Ex_5/Code/FreeRTOS/Question 2&3/FreeRTOS/include" --include_path="D:/RTES/EX 5/Nihal_T_Vishal_Raj_RTES_Ex_5/Nihal_T_Vishal_Raj_RTES_Ex_5/Code/FreeRTOS/Question 2&3/FreeRTOS/portable/CCS/ARM_CM4F" -g --gcc --define=ccs="ccs" --define=TARGET_IS_TM4C129_RA1 --define=PART_TM4C1294NCPDT --define=DEBUG --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="FreeRTOS/portable/CCS/ARM_CM4F/$(basename $(<F)).d_raw" --obj_directory="FreeRTOS/portable/CCS/ARM_CM4F" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

FreeRTOS/portable/CCS/ARM_CM4F/%.obj: ../FreeRTOS/portable/CCS/ARM_CM4F/%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --include_path="D:/RTES/EX 5/Nihal_T_Vishal_Raj_RTES_Ex_5/Nihal_T_Vishal_Raj_RTES_Ex_5/Code/FreeRTOS/Question 2&3/FreeRTOS/portable/CCS/ARM_CM4F" --include_path="D:/RTES/EX 5/Nihal_T_Vishal_Raj_RTES_Ex_5/Nihal_T_Vishal_Raj_RTES_Ex_5/Code/FreeRTOS/Question 2&3" --include_path="D:/RTES/EX 5/Nihal_T_Vishal_Raj_RTES_Ex_5/Nihal_T_Vishal_Raj_RTES_Ex_5/Code/FreeRTOS/Question 2&3/driverlib" --include_path="D:/RTES/EX 5/Nihal_T_Vishal_Raj_RTES_Ex_5/Nihal_T_Vishal_Raj_RTES_Ex_5/Code/FreeRTOS/Question 2&3/FreeRTOS/include" --include_path="D:/RTES/EX 5/Nihal_T_Vishal_Raj_RTES_Ex_5/Nihal_T_Vishal_Raj_RTES_Ex_5/Code/FreeRTOS/Question 2&3/FreeRTOS/portable/CCS/ARM_CM4F" -g --gcc --define=ccs="ccs" --define=TARGET_IS_TM4C129_RA1 --define=PART_TM4C1294NCPDT --define=DEBUG --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="FreeRTOS/portable/CCS/ARM_CM4F/$(basename $(<F)).d_raw" --obj_directory="FreeRTOS/portable/CCS/ARM_CM4F" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


