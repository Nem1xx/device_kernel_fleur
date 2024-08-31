ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
obj-$(CONFIG_CUSTOM_KERNEL_SENSORHUB)	+= sensorHub/
obj-$(CONFIG_MTK_HWMON)	    += hwmon/
ifeq ($(CONFIG_CUSTOM_KERNEL_ALS),y)
obj-$(CONFIG_CUSTOM_KERNEL_ALS) += alsps/
else
obj-$(CONFIG_CUSTOM_KERNEL_ALSPS) += alsps/
endif
ifeq ($(CONFIG_CUSTOM_KERNEL_ACCELGYRO),y)
obj-y += accelgyro/
else
obj-$(CONFIG_CUSTOM_KERNEL_ACCELEROMETER) += accelerometer/
obj-$(CONFIG_CUSTOM_KERNEL_GYROSCOPE) += gyroscope/
endif
obj-$(CONFIG_CUSTOM_KERNEL_MAGNETOMETER) += magnetometer/
obj-$(CONFIG_CUSTOM_KERNEL_BAROMETER) += barometer/
obj-$(CONFIG_CUSTOM_KERNEL_STEP_COUNTER) += step_counter/
obj-$(CONFIG_CUSTOM_KERNEL_SIGNIFICANT_MOTION_SENSOR) += step_counter/
obj-$(CONFIG_CUSTOM_KERNEL_SITUATION) += situation/
<<<<<<< HEAD
obj-$(CONFIG_CUSTOM_KERNEL_SENSOR_FUSION) += sensorfusion/
=======
#obj-$(CONFIG_CUSTOM_KERNEL_PEDOMETER)	+= pedometer/
obj-$(CONFIG_CUSTOM_KERNEL_SENSOR_FUSION)	+= sensorfusion/
#obj-$(CONFIG_MTK_KERNEL_GEOFENCE) += geofence/
obj-$(CONFIG_CUSTOM_KERNEL_BIOMETRIC_SENSOR) += biometric/
obj-$(CONFIG_CUSTOM_KERNEL_SENSOR_CAL) += sensor_cal/
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
else
obj-y += dummy.o
endif
