#pragma once
#include <Arduino.h>
enum class Workflow : uint8_t { Normal, TeachSetup, TeachMark };
enum class AppAction : uint8_t { Pick, RecordStart, RecordStop, RecordToggle, Play, Home, Button };
struct Action { AppAction type; uint8_t argument; };
bool dispatchAction(const Action& action);
bool onButtonPressed(uint8_t button);
bool executeAppCommand(char* const* tokens,uint8_t count,bool& success);
void initializeApplication();
Workflow currentWorkflow();
void printApplicationStatus();
