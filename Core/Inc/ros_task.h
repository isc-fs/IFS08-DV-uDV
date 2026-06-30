#ifndef ROS_TASK_H
#define ROS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief micro-ROS node main loop. Never returns.
 *
 * Called from StartDefaultTask() in freertos.c (after the CubeMX-managed
 * MX_USB_DEVICE_Init()). Sets up the USB-CDC transport, node, publishers,
 * subscriber, services and executor, then publishes IMU/steering/RES/AMI data
 * driven by imuQueueHandle. The body lives here rather than in the
 * CubeMX-regenerated freertos.c.
 */
void ros_task_run(void);

#ifdef __cplusplus
}
#endif

#endif /* ROS_TASK_H */
