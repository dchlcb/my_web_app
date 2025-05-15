#include "AlarmManager.h"

#include <sstream>

AlarmManager::AlarmManager() 
{
    pthread_mutex_init(&alarmMutex, nullptr);
}

AlarmManager::~AlarmManager() 
{
    pthread_mutex_destroy(&alarmMutex);
}

void AlarmManager::addAlarm(int code) 
{
    pthread_mutex_lock(&alarmMutex);
    alarms.insert(code);
    pthread_mutex_unlock(&alarmMutex);
}

void AlarmManager::removeAlarm(int code) 
{
    pthread_mutex_lock(&alarmMutex);
    alarms.erase(code);
    pthread_mutex_unlock(&alarmMutex);
}

std::string AlarmManager::getAlarmString() 
{
    pthread_mutex_lock(&alarmMutex);
    std::ostringstream oss;
    for (auto it = alarms.begin(); it != alarms.end(); ++it) 
    {
        if (it != alarms.begin()) 
        {
            oss << "-";
        }
        oss << *it;
    }
    pthread_mutex_unlock(&alarmMutex);
    return oss.str();
}