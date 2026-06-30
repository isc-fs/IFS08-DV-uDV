#ifndef AMI_TASK_H
#define AMI_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS entry point for the AMI task.
 *
 * Created and scheduled by MX_FREERTOS_Init() in freertos.c; the body lives
 * here so the CubeMX-generated file stays thin. Drives the WS2812 LED chain to
 * reflect the current mission index.
 */
void StartAmiTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* AMI_TASK_H */
