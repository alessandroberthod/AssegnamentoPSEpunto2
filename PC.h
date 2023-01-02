// PC.h

#ifndef PC_H
#define PC_H

#include <vector>
#include <mutex>
#include <condition_variable>
#include "Coordinate.h"

typedef Coordinate datatype;

class PC {
public:
    PC(int capacity);
    void append(datatype v);
    datatype take();
    
private:
    std::vector<datatype> buffer_;
    int capacity_;

    int front_;
    int rear_;
    int count_;

    std::mutex mutex_;
    std::condition_variable not_full_;
    std::condition_variable not_empty_;

};

#endif
