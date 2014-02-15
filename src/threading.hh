/*
 * FILE: threading.hh
 * DESCRIPTION:
 * Here a set of functions defined,
 * to provide safe multi-threading environment.
 * All functionality of random numbers generation also defined here.
 */

#ifndef THREADING_HH_
#define THREADING_HH_
#include <stdexcept>
#include <pthread.h>

#include <randoma.h>
#include "defs.hh"

//maximal number of created threads
#ifndef NTHREADS
#define NTHREADS (4) //default value
#endif //NTHREADS

//number of pixels what rendering thread will take to serve at once
#define THREAD_SERV_CHUNK (16000u)

/* Wrapping class for random numbers generator from randoma library.
 * Used to seed new instance of underlying class in thread-safe manner.
 */
class Randomizer : public CRandomSFMTA0
{
public:
  Randomizer()
    : CRandomSFMTA0(seed())
  {
  }

private:

  /* Thread-safe function.
   * Used to generate random integer seed number.
   */
  static int seed();
}; //class Randomizer

/* Thread-safe function.
 * The caller thread gets unique id in continuous integer range starting from 0.
 * Each thread may call this function as many times as it wants, and always
 * will retrieve it's own unique id.
 * This id used by the thread to access it's own exclusive entry
 * in shared data arrays.
 */
uint threadId();


/* Thread-safe function.
 * Returns random number from the range [minVal, maxVal].
 * The calling thread should specify its tid, as returned from threadId().
 */
double randomf(uint tid, double minVal, double maxVal);

/* class Lock.
 * C++ wrapper for pthread_mutex objects.
 * Provides exclusive access in multi-threaded environment.
 */
class Lock
{
public:
  Lock() throw(std::runtime_error)
  {
    if (pthread_mutex_init(&_mutex, NULL))
      throw std::runtime_error("pthread_mutex_init failed.");
  }

  /* Locks on current lock object.
   * Returns true on error.
   */
  bool lock()
  {
    if (pthread_mutex_lock(&_mutex))
    {
      cerr << "System error: [pthread_mutex_lock].\n";
      return true;
    }
    return false;
  }

  /* Unlocks current lock object.
   * Returns true on error.
   */
  bool unlock()
  {
    if (pthread_mutex_unlock(&_mutex))
    {
      cerr << "System error: [pthread_mutex_unlock].\n";
      return true;
    }
    return false;
  }

  ~Lock()
  {
    pthread_mutex_destroy(&_mutex);
  }

private:
  pthread_mutex_t _mutex;

  //restrict instance copying
  Lock(const Lock &);
  Lock &operator=(const Lock&);

}; //class Lock

#endif // THREADING_HH_
