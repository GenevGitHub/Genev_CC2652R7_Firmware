################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Application/%.o: ../Application/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-armllvm_3.2.0.LTS/bin/tiarmclang.exe" -c @"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/config/build_components.opt" @"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/config/factory_config.opt"  -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"C:/Users/yuche/workspace_v12/genev_gGo_dashboard_CC2652R7" -I"C:/Users/yuche/workspace_v12/genev_gGo_dashboard_CC2652R7/Release" -I"C:/Users/yuche/workspace_v12/genev_gGo_dashboard_CC2652R7/Application" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/controller/cc26xx/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/rom" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/common/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/icall/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/hal/src/target/_common" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/common/cc26xx/npi/stack" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/hal/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/heapmgr" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/profiles/dev_info" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/profiles/simple_profile" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/icall/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/npi/src" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/osal/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/services/src/saddr" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/services/src/sdata" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/common/nv" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/common/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/icall/src" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/bleapp/profiles/health_thermometer" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/bleapp/services/health_thermometer" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/common/cc26xx/rcosc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/ble5stack/npi/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/devices/cc13x2x7_cc26x2x7" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/kernel/tirtos7/packages" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_7_40_00_77/source/ti/posix/ticlang" -DFLASH_ROM_BUILD -DNVOCMP_NWSAMEITEM=1 -DHEAPMGR_CONFIG=0x80 -DHEAPMGR_SIZE=0x0 -gdwarf-3 -march=armv7e-m -MMD -MP -MF"Application/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/yuche/workspace_v12/genev_gGo_dashboard_CC2652R7/Release/syscfg" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


