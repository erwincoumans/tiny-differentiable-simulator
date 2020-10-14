// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TINY_RESIZABLE_POOL_H
#define TINY_RESIZABLE_POOL_H

#include <assert.h>
#include <vector>

enum { B3_POOL_HANDLE_TERMINAL_FREE = -1, B3_POOL_HANDLE_TERMINAL_USED = -2 };

template <typename U>
struct TinyPoolBodyHandle : public U {
  int m_nextFreeHandle;
  void setNextFree(int next) { m_nextFreeHandle = next; }
  int getNextFree() const { return m_nextFreeHandle; }
};

template <typename T>
class TinyResizablePool {
 protected:
  std::vector<T> m_bodyHandles;
  int m_numUsedHandles;   // number of active handles
  int m_firstFreeHandle;  // free handles list

  T* getHandleInternal(int handle) { return &m_bodyHandles[handle]; }
  const T* getHandleInternal(int handle) const {
    return &m_bodyHandles[handle];
  }

 public:
  TinyResizablePool() { init_handles(); }

  virtual ~TinyResizablePool() { exit_handles(); }
  /// handle management

  int get_num_handles() const { return m_bodyHandles.size(); }

  void get_used_handles(std::vector<int>& usedHandles) const {
    for (int i = 0; i < m_bodyHandles.size(); i++) {
      if (m_bodyHandles[i].getNextFree() == B3_POOL_HANDLE_TERMINAL_USED) {
        usedHandles.push_back(i);
      }
    }
  }

  T* get_handle(int handle) {
    assert(handle >= 0);
    assert(handle < m_bodyHandles.size());
    if ((handle < 0) || (handle >= m_bodyHandles.size())) {
      return 0;
    }

    if (m_bodyHandles[handle].getNextFree() == B3_POOL_HANDLE_TERMINAL_USED) {
      return &m_bodyHandles[handle];
    }
    return 0;
  }
  const T* get_handle(int handle) const {
    assert(handle >= 0);
    assert(handle < m_bodyHandles.size());
    if ((handle < 0) || (handle >= m_bodyHandles.size())) {
      return 0;
    }

    if (m_bodyHandles[handle].getNextFree() == B3_POOL_HANDLE_TERMINAL_USED) {
      return &m_bodyHandles[handle];
    }
    return 0;
  }

  void increase_handle_capacity(int extraCapacity) {
    int curCapacity = m_bodyHandles.size();
    // assert(curCapacity == m_numUsedHandles);
    int newCapacity = curCapacity + extraCapacity;
    m_bodyHandles.resize(newCapacity);

    {
      for (int i = curCapacity; i < newCapacity; i++)
        m_bodyHandles[i].setNextFree(i + 1);

      m_bodyHandles[newCapacity - 1].setNextFree(-1);
    }
    m_firstFreeHandle = curCapacity;
  }
  void init_handles() {
    m_numUsedHandles = 0;
    m_firstFreeHandle = -1;

    increase_handle_capacity(1);
  }

  void exit_handles() {
    m_bodyHandles.resize(0);
    m_firstFreeHandle = -1;
    m_numUsedHandles = 0;
  }

  int alloc_handle() {
    assert(m_firstFreeHandle >= 0);

    int handle = m_firstFreeHandle;
    m_firstFreeHandle = getHandleInternal(handle)->getNextFree();
    m_numUsedHandles++;

    if (m_firstFreeHandle < 0) {
      // int curCapacity = m_bodyHandles.size();
      int additionalCapacity = m_bodyHandles.size();
      increase_handle_capacity(additionalCapacity);

      getHandleInternal(handle)->setNextFree(m_firstFreeHandle);
    }
    getHandleInternal(handle)->setNextFree(B3_POOL_HANDLE_TERMINAL_USED);
    getHandleInternal(handle)->clear();
    return handle;
  }

  void free_handle(int handle) {
    assert(handle >= 0);

    if (m_bodyHandles[handle].getNextFree() == B3_POOL_HANDLE_TERMINAL_USED) {
      getHandleInternal(handle)->clear();
      getHandleInternal(handle)->setNextFree(m_firstFreeHandle);
      m_firstFreeHandle = handle;
      m_numUsedHandles--;
    }
  }
};
/// end handle management

#endif  // TINY_RESIZABLE_POOL_H
