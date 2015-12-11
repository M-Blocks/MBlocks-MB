#ifndef LIGHTEVENT_H_
#define LIGHTEVENT_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	TRACKER_PRIMITIVE_START_SEQUENCE,
	TRACKER_PRIMITIVE_TIMER_EXPIRED,
	TRACKER_PRIMITIVE_EVENT_COMPLETE,
	TRACKER_PRIMITIVE_EVENT_FAILURE,
} trackerPrimitive_t;

typedef enum {
	LIGHT_TRACKER_EVENT_COMPLETE,
} trackerEvent_t;

bool lightTracker_init();

bool lightTracker_startLightTracker(uint16_t bldcSpeed_rpm,	uint16_t brakeCurrent_mA, uint16_t brakeTime_ms, uint16_t threshold, app_sched_event_handler_t lightTrackerEventHandler);

bool lightTracker_readAmbientLight(int *ambientReadings);
bool lightTracker_getConfiguration(int *config, vectorFloat_t gravity);

#endif /* LIGHTEVENT_H_ */
