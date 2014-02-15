/*
 * FILE: threading.cc
 * DESCRIPTION:
 * Thread-safe environment functions.
 * Implementation of some functions, that defined in threading.hh.
 */
#include <cstdlib>
#include <ctime>

#include <map>

#include "threading.hh"

//hold separate randomizer instance for each thread
static Randomizer rnd[NTHREADS];

int Randomizer::seed()
{
  static bool call_srand = true;
  static Lock lock;

  int seed;

  lock.lock(); //locking access to system rand()
  if (call_srand)
  { //seeding system randomizer once per runtime
    srand(time(NULL));
    call_srand = false;
  }
  seed = rand(); //get random seed
  lock.unlock();
  return seed;
}

uint threadId()
{
  typedef std::map<pthread_t, uint> ThreadsMapping;
  //mapping of system thread ids into ordered integer range {0, 1, 2 ...}
  static ThreadsMapping threadsMapping;
  static Lock lock; //lock for threadsMapping exclusive access
  static uint nextId = 0; //next available id for new thread

  uint myId = -1;
  lock.lock(); //exclusive access to threadsMapping

  pthread_t tid = pthread_self(); //get system thread id
  ThreadsMapping::iterator it = threadsMapping.find(tid);
  if (it == threadsMapping.end())
  { //not yet mapped: create new entry
    threadsMapping.insert(std::make_pair(tid, nextId));
    myId = nextId++;
  }
  //already mapped: return the mapped value
  else myId = it->second;

  lock.unlock(); //release threadsMapping

  assert(myId < nextId);
  return myId;
}

double randomf(uint tid, double minVal, double maxVal)
{
  //using exclusive randomizer instance of the caller thread
  return minVal + rnd[tid].Random() * (maxVal - minVal);
}
