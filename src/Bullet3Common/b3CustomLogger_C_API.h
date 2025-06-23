#pragma once

#ifdef __cplusplus
extern "C" {
#endif

	// C-compatible functions (no c++ types)
	void b3LogEvent(const char* message);
	void b3CloseLog();

#ifdef __cplusplus
}
#endif