#include "b3CustomLogger.h"
#include <iostream>

std::ofstream b3CustomLogger::logFile;

void b3CustomLogger::logEvent(const std::string& message) {
	if (!logFile.is_open()) {
		logFile.open("simulation_log.txt", std::ios::app);

		if (!logFile.is_open()) {
			std::cerr << "Failed to open log file.\n";
			std::cerr << "[LOG] " << message << std::endl;
			return;
		}

		logFile << "=== Sim log started ===\n";
	}

	logFile << message << std::endl;

}

void b3CustomLogger::closeLog() {
	if (logFile.is_open()) {
		logFile << "=== Sim log ended ===\n";
		logFile.close();
	}
}