#include "vex.h"
#include "DOONLIB/central/doon_api.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller = controller(primary);

void vexcodeInit(void) {}