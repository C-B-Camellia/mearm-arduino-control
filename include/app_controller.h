#pragma once
#include <Arduino.h>
enum class Workflow : uint8_t { Normal, TeachSetup, TeachMark };
#include "business_commands.h"
struct Action { AppAction type; uint8_t argument; };
bool dispatchAction(const Action& action);
bool executeAppCommand(char* const* tokens,uint8_t count,bool& success);
void initializeApplication();
Workflow currentWorkflow();
void printApplicationStatus();
