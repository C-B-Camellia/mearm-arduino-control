#pragma once
#include "arm.h"
constexpr uint8_t RECORD_CAPACITY=80;
constexpr uint16_t RECORD_INTERVAL_MS=150;
struct RecordedFrame { uint8_t angles[JointCount]; uint16_t timeMs; };
enum class RecordState : uint8_t { Empty, Recording, Ready };
struct RecordingStatus { RecordState state; uint8_t count; uint16_t duration; };
bool startRecording(uint32_t now);
bool stopRecording(uint32_t now);
void updateRecording(uint32_t now,const Pose& pose);
bool isRecording();
bool hasRecording();
RecordingStatus recordingStatus();
bool getRecordedFrame(uint8_t index,RecordedFrame& out);
void clearRecording();
