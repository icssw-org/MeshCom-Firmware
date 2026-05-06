// -----------------------------------------------------------------------------
// DEBUG
// -----------------------------------------------------------------------------

// If not on PIO or not defined in platformio.ini
#ifndef DO_DEBUG
// Debug output set to 0 to disable app debug output
#define DO_DEBUG 0
#endif

#if DO_DEBUG > 0
#define DEBUG_MSG(tag, ...)                \
	do                                   \
	{                                    \
		if (tag)                         \
        Serial.print("");\
		Serial.printf("  <%s> ", tag); \
		Serial.printf(__VA_ARGS__);      \
		Serial.printf("\n");             \
	} while (0)

#define DEBUG_MSG_VAL(tag, val, ...)                \
	do                                   \
	{                                    \
		if (tag)                         \
        Serial.print("");\
		Serial.printf("  <%s> ", tag); \
		Serial.printf(__VA_ARGS__);      \
		Serial.printf("  <%d> ", val); \
		Serial.printf("\n");             \
	} while (0)

#define DEBUG_MSG_TXT(tag, txt, ...)                \
	do                                   \
	{                                    \
		if (tag)                         \
        Serial.print("");\
		Serial.printf("  <%s> ", tag); \
		Serial.printf(__VA_ARGS__);      \
		Serial.printf("  %s ", txt); \
		Serial.printf("\n");             \
	} while (0)

#else
#define DEBUG_MSG(...)
#define DEBUG_MSG_VAL(...)
#define DEBUG_MSG_TXT(...)
#endif