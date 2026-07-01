/**
 ******************************************************************************
 * @file    ros_task.c
 * @brief   micro-ROS node task (publishers, subscriber, services, executor).
 *
 * Body extracted verbatim from freertos.c (StartDefaultTask) so the
 * CubeMX-regenerated file stays thin. StartDefaultTask still owns the
 * CubeMX-managed MX_USB_DEVICE_Init() call and then hands off to ros_task_run().
 ******************************************************************************
 */
#include "ros_task.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_srvs/srv/empty.h>
#include <std_srvs/srv/set_bool.h>

#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "imu_service.h"      /* imu_sample_t */
#include "can_globals.h"      /* can_c_get_* accessors */
#include "as_state.h"         /* ros_get_as_state (raw ASState) */
#include "dwt_time.h"         /* dwt_micros */
#include "imu_task.h"         /* imu_debug_status */
#include "dv_interface.h"     /* stock-typed pipeline interface: dv/status + ctrl/cmd */

#define G_TO_MS2   9.80665f
#define DPS_TO_RAD (float)(M_PI / 180.0)
#define TIME_SYNC_TIMEOUT_MS  1000
#define TIME_SYNC_INTERVAL    4000  // re-sync every N samples (~10s at 400Hz)
#define SLOW_PUB_INTERVAL     40    // publish steering/RES/AMI every 40 IMU samples (~10 Hz)
#define DL_TX_INTERVAL_MS     100   // Data Logger TX period
#define RES_TIMEOUT_MS        150   // RES PDO timeout (expect every 30 ms)
#define STATE_DUMP_INTERVAL   200   // /debug state heartbeat: every 200 IMU samples (~2 Hz at 400 Hz)

/* RTOS queues created in freertos.c (MX_FREERTOS_Init). */
extern osMessageQueueId_t imuQueueHandle;
extern osMessageQueueId_t debugQueueHandle;

/* micro-ROS transport + allocator hooks (defined in micro_ros_stm32cubemx_utils). */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

/* --------------------------------------------------------------------------
 * /cmd_test subscriber callback: toggle OK_STATUS LED (PD14)
 * -------------------------------------------------------------------------- */
static void cmd_test_callback(const void *msgin)
{
  (void)msgin;
  HAL_GPIO_TogglePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin);
}

// --- Service callback for /activate_steering ---

void activate_steering_callback(const void * req, void * res)
{
  // Casteamos los punteros a los tipos correctos de SetBool
  const std_srvs__srv__SetBool_Request * request = (const std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response * response = (std_srvs__srv__SetBool_Response *)res;

  char srv_msg[128];

  // Evaluamos el bool que nos llega en request->data
  if (request->data) {
    snprintf(srv_msg, sizeof(srv_msg), "debug: Servicio llamado con TRUE. Activando modo especial.");
    HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);

    // Ejemplo: Encender un LED o activar una bandera interna
    //HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);
  } else {
    snprintf(srv_msg, sizeof(srv_msg), "debug: Servicio llamado con FALSE. Desactivando modo especial.");
    //HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_RESET);
  }

  // Enviamos el mensaje al queue de debug
  osMessageQueuePut(debugQueueHandle, &srv_msg, 0, 0);

  // Respondemos al cliente de ROS 2 que todo ha salido bien
  response->success = true;
  // Opcional: rellenar el mensaje de retorno (requiere inicializar el string de la respuesta si se usa)
  // rosidl_runtime_c__String__assign(&response->message, "OK");
}

// --- Service callback for /force_ebs ---

void force_ebs_callback(const void * req, void * res)
{
  // Casteamos los punteros a los tipos correctos de SetBool
  const std_srvs__srv__SetBool_Request * request = (const std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response * response = (std_srvs__srv__SetBool_Response *)res;

  char srv_msg[128];

  // Evaluamos el bool que nos llega en request->data
  if (request->data) {
    snprintf(srv_msg, sizeof(srv_msg), "debug: Forzando apertura de EBS");
    HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET); // Forzar EBS abierto
    HAL_GPIO_WritePin(D1_GPIO_Port, D2_Pin, GPIO_PIN_SET); // Forzar EBS abierto
  } else {
    snprintf(srv_msg, sizeof(srv_msg), "debug: Vuelta a estado normal");
    HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D1_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
  }

  // Enviamos el mensaje al queue de debug
  osMessageQueuePut(debugQueueHandle, &srv_msg, 0, 0);

  // Respondemos al cliente de ROS 2 que todo ha salido bien
  response->success = true;
}

/* --------------------------------------------------------------------------
 * Stock-typed pipeline interface subscribers (see dv_interface.h).
 * -------------------------------------------------------------------------- */

/* /ctrl/cmd (geometry_msgs/Twist): the DVPC's normalised drive command.
 * linear.x = throttle [-1,1], angular.z = steering [-1,1]. Stored (clamped)
 * for AppTask, which actuates it only while AS Driving. The publisher is
 * BEST_EFFORT, so this subscription MUST be best-effort too or DDS
 * request-vs-offered matching silently drops every command. */
static void ctrl_cmd_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  ros_set_ctrl_cmd_norm((float)m->linear.x, (float)m->angular.z,
                        osKernelGetTickCount());
}

/* /dv/status (std_msgs/UInt8): the DVPC lifecycle byte — the prepare/run
 * handshake + the DVPC liveness heartbeat. AppTask gates AS Ready->Driving
 * on DV_STATUS_READY and reacts to FINISHED / EMERGENCY / FAILED / staleness. */
static void dv_status_callback(const void *msgin)
{
  const std_msgs__msg__UInt8 *m = (const std_msgs__msg__UInt8 *)msgin;
  ros_set_dv_status(m->data, osKernelGetTickCount());
}

/* Map raw ASState (see as_state.h: OFF=0 READY=1 DRIVING=2 EMERGENCY=3
   FINISHED=4) to a human-readable name for the /debug state line. */
static const char *as_state_name(uint8_t s)
{
  switch (s)
  {
    case 0:  return "OFF";
    case 1:  return "READY";
    case 2:  return "DRIVING";
    case 3:  return "EMERGENCY";
    case 4:  return "FINISHED";
    default: return "UNKNOWN";
  }
}

/* Map raw EBSInitState (see ebs_manager.hpp) to a human-readable name. */
static const char *ebs_init_name(uint8_t s)
{
  switch (s)
  {
    case 0:  return "Start";
    case 1:  return "WaitLow";
    case 2:  return "CheckPressure";
    case 3:  return "WaitTS";
    case 4:  return "CheckActuator1";
    case 5:  return "WaitInterActuatorCheck";
    case 6:  return "CheckActuator2";
    case 7:  return "Failed";
    case 8:  return "Done";
    default: return "Unknown";
  }
}

/* Map can_c_get_res_status() codes to a short RES status name. */
static const char *res_status_name(int32_t s)
{
  switch (s)
  {
    case  0: return "OK";       /* received, no e-stop / go */
    case  1: return "ESTOP";    /* emergency stop asserted  */
    case  2: return "GO";       /* go signal active         */
    case -1: return "TIMEOUT";  /* RES PDO stale            */
    case -2: return "NONE";     /* never received           */
    default: return "?";
  }
}

/* Build the compact grouped one-line state-machine snapshot for /debug, with
   ` || ` group separators and `key:val` fields. When the AS state changed since
   the last publish (prev != cur) it is shown as "AS PREV->CUR", else "AS CUR".
   sig is packed with the AS_SIG_* bits from as_state.h; res is the raw
   can_c_get_res_status() code. */
static void format_state_debug(char *buf, size_t n, uint8_t prev_as,
                               uint8_t as, uint16_t sig, uint8_t ebs, int32_t res)
{
  char as_tok[48];
  if (prev_as != as)
    snprintf(as_tok, sizeof(as_tok), "AS %s->%s", as_state_name(prev_as), as_state_name(as));
  else
    snprintf(as_tok, sizeof(as_tok), "AS %s", as_state_name(as));

  snprintf(buf, n,
    "%s || ASMS:%s TS:%s SDC:%s EBS:%s ABS:%s || "
    "brakes:%s mission:%s R2D:%s motion:%s finished:%s || RES:%s || EBSinit:%s",
    as_tok,
    (sig & AS_SIG_ASMS_ON)        ? "on"         : "off",
    (sig & AS_SIG_TS_ACTIVE)      ? "on"         : "off",
    (sig & AS_SIG_SDC_RES_OPEN)   ? "open"       : "closed",
    (sig & AS_SIG_EBS_ACTIVATED)  ? "on"         : "off",
    (sig & AS_SIG_ABS_CHECKS_OK)  ? "ok"         : "fail",
    (sig & AS_SIG_BRAKES_ENGAGED) ? "on"         : "off",
    (sig & AS_SIG_MISSION_SEL)    ? "set"        : "unset",
    (sig & AS_SIG_R2D)            ? "on"         : "off",
    (sig & AS_SIG_STANDSTILL)     ? "standstill" : "moving",
    (sig & AS_SIG_MISSION_DONE)   ? "yes"        : "no",
    res_status_name(res),
    ebs_init_name(ebs));
}

void ros_task_run(void)
{
  // Enable DWT cycle counter for high-res timestamps
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Wait for USB CDC to enumerate on the host
  osDelay(2000);

  // micro-ROS custom transport (USB CDC, HDLC framing)
  rmw_uros_set_custom_transport(
    true,
    NULL,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  // Set FreeRTOS-compatible allocators for micro-ROS
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    for (;;) { osDelay(1000); }
  }

  // micro-ROS node
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "cubemx_node", "", &support);

  // Time synchronization — up to 5 attempts, continue regardless.
  for (int ts_try = 0; ts_try < 5 && !rmw_uros_epoch_synchronized(); ts_try++) {
    rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
    if (!rmw_uros_epoch_synchronized()) {
      osDelay(100);
    }
  }
  int64_t sync_epoch_ns = rmw_uros_epoch_nanos();
  uint64_t sync_dwt_us = dwt_micros();
  int64_t epoch_offset_ns = sync_epoch_ns - (int64_t)sync_dwt_us * 1000LL;

  // --- Publishers ---

  // IMU publisher (best-effort QoS for maximum throughput)
  rcl_publisher_t imu_pub;
  rclc_publisher_init_best_effort(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu");

  // IMU debug status publisher
  rcl_publisher_t imu_debug_pub;
  std_msgs__msg__Int32 debug_msg;
  debug_msg.data = 0;
  rclc_publisher_init_default(
    &imu_debug_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "imu/status");

  // Steering angle publisher (~10 Hz) — radians, as expected by pipeline
  rcl_publisher_t steering_pub;
  std_msgs__msg__Float32 steering_msg;
  steering_msg.data = 0.0f;
  rclc_publisher_init_best_effort(
    &steering_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "steering_angle");

  // Motor RPM publisher (~10 Hz) — placeholder: no RPM sensor on uDV
  rcl_publisher_t motor_rpm_pub;
  std_msgs__msg__Float32 motor_rpm_msg;
  motor_rpm_msg.data = 0.0f;
  rclc_publisher_init_best_effort(
    &motor_rpm_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_rpm");

  // Steering feedback publisher (20 Hz)
  // data[0] = angle_actual (deg) — LWS physical angle
  // data[1] = angle_target (deg) — last commanded angle echoed by controller
  // data[2] = angle_motor  (deg) — stepper position calculated by steps
  rcl_publisher_t steering_fb_pub;
  std_msgs__msg__Float32MultiArray steering_fb_msg;
  static float steering_fb_data[3] = {0.0f, 0.0f, 0.0f};
  memset(&steering_fb_msg, 0, sizeof(steering_fb_msg));
  steering_fb_msg.data.data     = steering_fb_data;
  steering_fb_msg.data.size     = 3;
  steering_fb_msg.data.capacity = 3;
  rclc_publisher_init_best_effort(
    &steering_fb_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "steering/feedback");

  // RES status publisher (~10 Hz)
  rcl_publisher_t res_pub;
  std_msgs__msg__Int32 res_msg;
  res_msg.data = 0;
  rclc_publisher_init_best_effort(
    &res_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "res/status");

  // AMI mission publisher (~10 Hz)
  rcl_publisher_t ami_pub;
  std_msgs__msg__Int32 ami_msg;
  ami_msg.data = -1;
  rclc_publisher_init_best_effort(
    &ami_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ami/mission");

  // GO signal publisher (~10 Hz): 0=no GO, 1=GO active
  rcl_publisher_t go_pub;
  std_msgs__msg__Int32 go_msg;
  go_msg.data = 0;
  rclc_publisher_init_best_effort(
    &go_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "res/go");

  // Debug string publisher
  rcl_publisher_t debug_pub;
  std_msgs__msg__String debug_str_msg;
  memset(&debug_str_msg, 0, sizeof(debug_str_msg));
  rclc_publisher_init_default(
    &debug_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "debug");

  // AS state publisher (~10 Hz) — FS-Rules T14.9 byte, mirrors CAN 0x100.
  // The DV pipeline (mission_control) reconciles its lifecycle off this
  // topic and also treats it as the uDV liveness heartbeat, so it must be
  // published at a steady cadence (not edge-only). Best-effort/volatile
  // like the other state heartbeats — the pipeline-side subscription must
  // match this QoS (best-effort), NOT reliable+transient-local.
  rcl_publisher_t assi_pub;
  std_msgs__msg__UInt8 assi_msg;
  assi_msg.data = 0;
  rclc_publisher_init_best_effort(
    &assi_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "assi/state");

  // Raw AS-state publisher (~10 Hz) — the AS state machine state itself
  // (ASState: OFF=0 READY=1 DRIVING=2 EMERGENCY=3 FINISHED=4), distinct
  // from the ASSI-indicator code on /assi/state. General-purpose AS-state
  // signal for the pipeline / telemetry / debugging.
  rcl_publisher_t as_state_pub;
  std_msgs__msg__UInt8 as_state_msg;
  as_state_msg.data = 0;
  rclc_publisher_init_best_effort(
    &as_state_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "as_state");

  // --- Subscriber ---

  // /cmd_test subscriber (toggle LED on receive)
  rcl_subscription_t cmd_test_sub;
  std_msgs__msg__Int32 cmd_test_msg;
  rclc_subscription_init_default(
    &cmd_test_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cmd_test");

  // /dv/status subscriber — pipeline lifecycle byte. RELIABLE (default)
  // matches the pipeline's latched RELIABLE+TRANSIENT_LOCAL publisher and
  // guarantees we never miss a status transition.
  rcl_subscription_t dv_status_sub;
  std_msgs__msg__UInt8 dv_status_msg;
  rclc_subscription_init_default(
    &dv_status_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    DV_TOPIC_DV_STATUS);

  // /ctrl/cmd subscriber — normalised drive command. MUST be BEST_EFFORT:
  // the pipeline publishes it best-effort, and a reliable reader would fail
  // DDS QoS matching and receive nothing (car sits still, no error).
  rcl_subscription_t ctrl_cmd_sub;
  geometry_msgs__msg__Twist ctrl_cmd_msg;
  rclc_subscription_init_best_effort(
    &ctrl_cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    DV_TOPIC_CTRL_CMD);

  // --- SERVICIOS ---
  rcl_service_t Activate_stearing;
  std_srvs__srv__SetBool_Request act_steer_srv_req = {0};
  std_srvs__srv__SetBool_Response act_steer_srv_res = {0};

  rclc_service_init_default(
    &Activate_stearing, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "activate_steering"
  );

  rcl_service_t Force_EBS;
  std_srvs__srv__SetBool_Request force_ebs_srv_req = {0};
  std_srvs__srv__SetBool_Response force_ebs_srv_res = {0};

  rclc_service_init_default(
    &Force_EBS, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "force_ebs"
  );

  // --- Executor ---
  // One handle per subscription + service: cmd_test, dv/status, ctrl/cmd,
  // activate_steering, force_ebs = 5.
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 5, &allocator);

  rclc_executor_add_subscription(
    &executor, &cmd_test_sub, &cmd_test_msg,
    &cmd_test_callback, ON_NEW_DATA);

  rclc_executor_add_subscription(
    &executor, &dv_status_sub, &dv_status_msg,
    &dv_status_callback, ON_NEW_DATA);

  rclc_executor_add_subscription(
    &executor, &ctrl_cmd_sub, &ctrl_cmd_msg,
    &ctrl_cmd_callback, ON_NEW_DATA);

  rclc_executor_add_service(
    &executor, &Activate_stearing, &act_steer_srv_req, &act_steer_srv_res,
    &activate_steering_callback);

  rclc_executor_add_service(
    &executor, &Force_EBS, &force_ebs_srv_req, &force_ebs_srv_res,
    &force_ebs_callback);

  // Prepare IMU message (set static fields once)
  sensor_msgs__msg__Imu imu_msg;
  memset(&imu_msg, 0, sizeof(imu_msg));
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

  // Orientation not available
  imu_msg.orientation_covariance[0] = -1.0;

  // BMI088 datasheet noise specs (diagonal covariance matrices)
  imu_msg.linear_acceleration_covariance[0] = 8.25e-4;
  imu_msg.linear_acceleration_covariance[4] = 8.25e-4;
  imu_msg.linear_acceleration_covariance[8] = 8.25e-4;
  imu_msg.angular_velocity_covariance[0] = 1.37e-5;
  imu_msg.angular_velocity_covariance[4] = 1.37e-5;
  imu_msg.angular_velocity_covariance[8] = 1.37e-5;

  uint16_t slow_pub_counter    = 0;
  uint16_t steering_fb_counter = 0;

  /* State-machine snapshot tracking for the /debug dump. Seed with the current
     values so we don't emit a spurious "change" on the first sample; the first
     heartbeat publishes the initial snapshot. */
  uint8_t  last_as_state = ros_get_as_state();
  uint16_t last_signals  = ros_get_state_signals();
  uint8_t  last_ebs      = ros_get_ebs_init_state();
  int32_t  last_res      = can_c_get_res_status(osKernelGetTickCount(), RES_TIMEOUT_MS);
  uint16_t state_dump_counter = STATE_DUMP_INTERVAL;  /* emit on the first sample */

  for (;;)
  {
    imu_sample_t sample;
    if (osMessageQueueGet(imuQueueHandle, &sample, NULL, osWaitForever) == osOK)
    {
      // Timestamp
      int64_t stamp_ns = epoch_offset_ns + (int64_t)sample.timestamp_us * 1000LL;
      imu_msg.header.stamp.sec = (int32_t)(stamp_ns / 1000000000LL);
      imu_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);

      // Linear acceleration: g -> m/s^2
      imu_msg.linear_acceleration.x = sample.imu.ax_g * G_TO_MS2;
      imu_msg.linear_acceleration.y = sample.imu.ay_g * G_TO_MS2;
      imu_msg.linear_acceleration.z = sample.imu.az_g * G_TO_MS2;

      // Angular velocity: dps -> rad/s
      imu_msg.angular_velocity.x = sample.imu.gx_dps * DPS_TO_RAD;
      imu_msg.angular_velocity.y = sample.imu.gy_dps * DPS_TO_RAD;
      imu_msg.angular_velocity.z = sample.imu.gz_dps * DPS_TO_RAD;

      (void)rcl_publish(&imu_pub, &imu_msg, NULL);

      // --- Full state-machine snapshot -> /debug (grouped one line; on change + ~2 Hz) ---
      uint8_t  as_now  = ros_get_as_state();
      uint16_t sig_now = ros_get_state_signals();
      uint8_t  ebs_now = ros_get_ebs_init_state();
      int32_t  res_now = can_c_get_res_status(osKernelGetTickCount(), RES_TIMEOUT_MS);
      bool changed  = (as_now != last_as_state) ||
                      (sig_now != last_signals) ||
                      (ebs_now != last_ebs) ||
                      (res_now != last_res);
      bool periodic = (++state_dump_counter >= STATE_DUMP_INTERVAL);
      if (changed || periodic)
      {
        char state_buf[256];
        format_state_debug(state_buf, sizeof(state_buf),
                           last_as_state, as_now, sig_now, ebs_now, res_now);
        rosidl_runtime_c__String__assign(&debug_str_msg.data, state_buf);
        (void)rcl_publish(&debug_pub, &debug_str_msg, NULL);
        last_as_state = as_now;
        last_signals  = sig_now;
        last_ebs      = ebs_now;
        last_res      = res_now;
        state_dump_counter = 0;
      }

      // --- Steering feedback at 20 Hz (every 20 IMU samples) ---
      if (++steering_fb_counter >= 20)
      {
        steering_fb_counter = 0;
        steering_fb_data[0] = can_c_get_steer_angle_actual();  // actual (deg)
        steering_fb_data[1] = can_c_get_steer_angle_target();  // target (deg)
        steering_fb_data[2] = can_c_get_steer_angle_motor();   // motor stepper (deg)
        (void)rcl_publish(&steering_fb_pub, &steering_fb_msg, NULL);
      }

      // --- Slow publishers (~10 Hz) ---
      if (++slow_pub_counter >= SLOW_PUB_INTERVAL)
      {
        slow_pub_counter = 0;

        // TODO: STEERING_RATIO must be measured on the physical car
        // (steering wheel degrees / front wheel degrees). Using 1.0 as
        // placeholder — EKF will overestimate yaw rate until corrected.
        #define STEERING_RATIO 1.0f
        steering_msg.data = can_c_get_steering_angle_deg()
                            * (float)(M_PI / 180.0)
                            / STEERING_RATIO;
        (void)rcl_publish(&steering_pub, &steering_msg, NULL);

        // Motor RPM — placeholder, no RPM sensor on uDV
        (void)rcl_publish(&motor_rpm_pub, &motor_rpm_msg, NULL);


        // RES status: 0=OK, 1=E-STOP, -1=TIMEOUT
        uint32_t now_tick = osKernelGetTickCount();
        res_msg.data = can_c_get_res_status(now_tick, RES_TIMEOUT_MS);
        (void)rcl_publish(&res_pub, &res_msg, NULL);

        // AMI mission index
        ami_msg.data = can_c_get_mission_index();
        (void)rcl_publish(&ami_pub, &ami_msg, NULL);

        // GO signal from RES
        go_msg.data = (int32_t)can_c_get_go_signal();
        (void)rcl_publish(&go_pub, &go_msg, NULL);

        // AS state (FS-Rules T14.9 byte) — mirrors the CAN 0x100 ASSI frame
        assi_msg.data = can_c_get_assi_status_code();
        (void)rcl_publish(&assi_pub, &assi_msg, NULL);

        // Raw AS state-machine state (ASState enum, not the ASSI code)
        as_state_msg.data = ros_get_as_state();
        (void)rcl_publish(&as_state_pub, &as_state_msg, NULL);

        // Spin executor to process /cmd_test subscription
        // Publish any queued debug messages from CAN service
        char dbgbuf[128];
        while (osMessageQueueGet(debugQueueHandle, &dbgbuf, NULL, 0) == osOK) {
          rosidl_runtime_c__String__assign(&debug_str_msg.data, dbgbuf);
          (void)rcl_publish(&debug_pub, &debug_str_msg, NULL);
        }

        // Heartbeat / periodic debug message
        static uint32_t debug_beat = 0;
        if (debug_beat % 10 == 0) {  // Publish every 10 iterations
          char beatbuf[64];
          snprintf(beatbuf, sizeof(beatbuf), "debug: heartbeat %u", debug_beat++);
          rosidl_runtime_c__String__assign(&debug_str_msg.data, beatbuf);
          (void)rcl_publish(&debug_pub, &debug_str_msg, NULL);
        }

        rclc_executor_spin_some(&executor, 0);
      }

      // Periodic time re-sync and IMU status publish (~every 10s)
      static uint16_t sync_counter = 0;
      if (++sync_counter >= TIME_SYNC_INTERVAL)
      {
        rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
        int64_t new_epoch_ns = rmw_uros_epoch_nanos();
        uint64_t new_dwt_us = dwt_micros();
        epoch_offset_ns = new_epoch_ns - (int64_t)new_dwt_us * 1000LL;

        extern volatile int32_t imu_debug_status;
        debug_msg.data = imu_debug_status;
        (void)rcl_publish(&imu_debug_pub, &debug_msg, NULL);
        sync_counter = 0;
      }
    }
  }
}
