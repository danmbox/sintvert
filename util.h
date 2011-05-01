// Copyright (C) 2010-2011 Dan Muresan
// Part of sintvert (http://danmbox.github.com/sintvert/)

#ifndef __SINTVERT__UTIL_H
#define __SINTVERT__UTIL_H

#include <pthread.h>

static void mutex_cleanup_routine (void *lock_) {
  pthread_mutex_unlock ((pthread_mutex_t *) lock_);
}
/// Locks a mutex and pushes a @c pthread_cleanup routine.
/// Must be matched with a <code>pthread_cleanup_pop (1)</code>
#define MUTEX_LOCK_WITH_CLEANUP( lock )                 \
  pthread_mutex_lock (lock);                            \
  pthread_cleanup_push (mutex_cleanup_routine, (lock))

#endif  // __SINTVERT__UTIL_H
