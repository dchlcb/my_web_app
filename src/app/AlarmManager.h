#ifndef ALARM_MANAGER_H
#define ALARM_MANAGER_H


#include <set>
#include <string>
#include <pthread.h>

class AlarmManager 
{
public:
    AlarmManager();
    ~AlarmManager();

    void addAlarm(int code);
    void removeAlarm(int code);
    std::string getAlarmString();

private:
    std::set<int> alarms;
    pthread_mutex_t alarmMutex;
};






#endif
