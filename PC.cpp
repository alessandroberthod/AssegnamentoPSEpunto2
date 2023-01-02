#include "PC.h"
#include <mutex>
using std::mutex;
using std::unique_lock;

PC::PC(int capacity) 
: capacity_(capacity), front_(0), rear_(0), count_(0), buffer_(capacity,{0,0}) 
{  
	buffer_.shrink_to_fit();
}

void PC::append(datatype v){
	unique_lock<std::mutex> mlock(mutex_);
	while( count_ == capacity_)
		not_full_.wait(mlock);
	buffer_.at(rear_) = v;
	rear_ = (rear_ + 1) % capacity_;
	++count_;
	mlock.unlock();
	not_empty_.notify_one();
}

datatype PC::take(){
	std::unique_lock<std::mutex> mlock(mutex_);
	while(count_ == 0)
		not_empty_.wait(mlock);

	datatype w = buffer_.at(front_);
    front_ = (front_ + 1) % capacity_;
    --count_;
    mlock.unlock();
    not_full_.notify_one();

    return w;
}