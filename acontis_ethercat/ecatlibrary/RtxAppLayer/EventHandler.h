#pragma once

/*---------------------------------------------------------------------------
 * EventHandler.h - Linux/pthread-based event and thread handling
 *                  Replaces Windows HANDLE/Event API
 *---------------------------------------------------------------------------*/

#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include "Def.h"

// Handle type for Linux - wraps a condition variable + mutex + flag
struct LinuxEvent {
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    bool            signaled;
    char            name[64];
};

// Shared event registry (simple approach for named events)
#define MAX_EVENTS 64

// Additional Windows compat types (HANDLE, LPDWORD, LPTHREAD_START_ROUTINE are in Def.h)
typedef unsigned long DWORD;
typedef void* LPVOID;
typedef void* LPSECURITY_ATTRIBUTES;
typedef unsigned long SIZE_T;

class EventRegistry {
public:
    static LinuxEvent* Find(const char* name) {
        pthread_mutex_lock(&registryMutex);
        for (int i = 0; i < count; i++) {
            if (strcmp(events[i].name, name) == 0) {
                pthread_mutex_unlock(&registryMutex);
                return &events[i];
            }
        }
        pthread_mutex_unlock(&registryMutex);
        return nullptr;
    }
    static LinuxEvent* Create(const char* name) {
        pthread_mutex_lock(&registryMutex);
        // Check if already exists
        for (int i = 0; i < count; i++) {
            if (strcmp(events[i].name, name) == 0) {
                pthread_mutex_unlock(&registryMutex);
                return &events[i];
            }
        }
        if (count >= MAX_EVENTS) {
            pthread_mutex_unlock(&registryMutex);
            return nullptr;
        }
        LinuxEvent* evt = &events[count++];
        pthread_mutex_init(&evt->mutex, nullptr);
        pthread_cond_init(&evt->cond, nullptr);
        evt->signaled = false;
        strncpy(evt->name, name, sizeof(evt->name) - 1);
        evt->name[sizeof(evt->name) - 1] = '\0';
        pthread_mutex_unlock(&registryMutex);
        return evt;
    }
    static void Init() {
        pthread_mutex_init(&registryMutex, nullptr);
        count = 0;
    }
private:
    static LinuxEvent events[MAX_EVENTS];
    static int count;
    static pthread_mutex_t registryMutex;
};

class EventHandler
{
public:
    static HANDLE Open(const char* strName)
    {
        LinuxEvent* evt = EventRegistry::Find(strName);
        if (!evt) evt = EventRegistry::Create(strName);
        return (HANDLE)evt;
    }
    static HANDLE Create(const char* strName)
    {
        return (HANDLE)EventRegistry::Create(strName);
    }
    static void Set(HANDLE hEvt)
    {
        LinuxEvent* evt = (LinuxEvent*)hEvt;
        if (!evt) return;
        pthread_mutex_lock(&evt->mutex);
        evt->signaled = true;
        pthread_cond_broadcast(&evt->cond);
        pthread_mutex_unlock(&evt->mutex);
    }
    static void Reset(HANDLE hEvt)
    {
        LinuxEvent* evt = (LinuxEvent*)hEvt;
        if (!evt) return;
        pthread_mutex_lock(&evt->mutex);
        evt->signaled = false;
        pthread_mutex_unlock(&evt->mutex);
    }
    static DWORD WaitFor(HANDLE hEvt, DWORD dwMilliSeconds)
    {
        LinuxEvent* evt = (LinuxEvent*)hEvt;
        if (!evt) return WAIT_TIMEOUT;

        pthread_mutex_lock(&evt->mutex);
        if (evt->signaled) {
            evt->signaled = false; // auto-reset
            pthread_mutex_unlock(&evt->mutex);
            return WAIT_OBJECT_0;
        }
        if (dwMilliSeconds == INFINITE) {
            while (!evt->signaled) {
                pthread_cond_wait(&evt->cond, &evt->mutex);
            }
            evt->signaled = false;
            pthread_mutex_unlock(&evt->mutex);
            return WAIT_OBJECT_0;
        }

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += dwMilliSeconds / 1000;
        ts.tv_nsec += (dwMilliSeconds % 1000) * 1000000;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        int rc = 0;
        while (!evt->signaled && rc == 0) {
            rc = pthread_cond_timedwait(&evt->cond, &evt->mutex, &ts);
        }
        if (evt->signaled) {
            evt->signaled = false;
            pthread_mutex_unlock(&evt->mutex);
            return WAIT_OBJECT_0;
        }
        pthread_mutex_unlock(&evt->mutex);
        return WAIT_TIMEOUT;
    }
    static void Close(HANDLE hEvt)
    {
        // Events are statically allocated; no-op
        (void)hEvt;
    }
};

class ThreadHandler
{
public:
    struct ThreadArg {
        void* (*func)(void*);
        void* param;
        bool  suspended;
        pthread_mutex_t suspendMutex;
        pthread_cond_t  suspendCond;
        pthread_t       tid;
    };

    static void* ThreadWrapper(void* arg) {
        ThreadArg* ta = (ThreadArg*)arg;
        // Check if created suspended
        pthread_mutex_lock(&ta->suspendMutex);
        while (ta->suspended) {
            pthread_cond_wait(&ta->suspendCond, &ta->suspendMutex);
        }
        pthread_mutex_unlock(&ta->suspendMutex);
        ta->func(ta->param);
        return nullptr;
    }

    static HANDLE Create(
        LPSECURITY_ATTRIBUTES /*lpAttr*/,
        SIZE_T /*dwStackSize*/,
        LPTHREAD_START_ROUTINE lpStartAddress,
        LPVOID lpParameter,
        DWORD dwCreationFlags,
        LPDWORD /*lpThreadId*/)
    {
        ThreadArg* ta = new ThreadArg();
        ta->func = (void*(*)(void*))lpStartAddress;
        ta->param = lpParameter;
        ta->suspended = (dwCreationFlags & CREATE_SUSPENDED) != 0;
        pthread_mutex_init(&ta->suspendMutex, nullptr);
        pthread_cond_init(&ta->suspendCond, nullptr);

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

        int ret = pthread_create(&ta->tid, &attr, ThreadWrapper, ta);
        pthread_attr_destroy(&attr);
        if (ret != 0) {
            delete ta;
            return nullptr;
        }
        return (HANDLE)ta;
    }

    static void SetPriority(HANDLE thread, int priority)
    {
        (void)thread;
        (void)priority;
        // On Linux, setting thread priority requires real-time scheduling
        // which needs root. Skip for now.
    }

    static void Suspend(HANDLE thread)
    {
        ThreadArg* ta = (ThreadArg*)thread;
        if (!ta) return;
        pthread_mutex_lock(&ta->suspendMutex);
        ta->suspended = true;
        pthread_mutex_unlock(&ta->suspendMutex);
    }

    static void Resume(HANDLE thread)
    {
        ThreadArg* ta = (ThreadArg*)thread;
        if (!ta) return;
        pthread_mutex_lock(&ta->suspendMutex);
        ta->suspended = false;
        pthread_cond_signal(&ta->suspendCond);
        pthread_mutex_unlock(&ta->suspendMutex);
    }

    static bool Join(HANDLE thread, DWORD dwMilliSeconds)
    {
        ThreadArg* ta = (ThreadArg*)thread;
        if (!ta) return true;
        if (dwMilliSeconds == INFINITE) {
            pthread_join(ta->tid, nullptr);
            return true;
        }
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += dwMilliSeconds / 1000;
        ts.tv_nsec += (dwMilliSeconds % 1000) * 1000000;
        if (ts.tv_nsec >= 1000000000) { ts.tv_sec++; ts.tv_nsec -= 1000000000; }
        int rc = pthread_timedjoin_np(ta->tid, nullptr, &ts);
        return (rc == 0);
    }

    static void Delete(HANDLE thread)
    {
        ThreadArg* ta = (ThreadArg*)thread;
        if (!ta) return;
        pthread_mutex_destroy(&ta->suspendMutex);
        pthread_cond_destroy(&ta->suspendCond);
        delete ta;
    }
};
