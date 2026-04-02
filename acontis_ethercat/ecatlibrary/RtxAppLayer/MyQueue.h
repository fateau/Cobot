#pragma once
#include <iostream>
#include <pthread.h>
#include <stdexcept>
using namespace std;

template <class T>
class MyQueue
{
public:
	MyQueue(int maxSize);
	~MyQueue(void);

	//從尾端加入
	int  enqueue(T node);
	
	//從頭拿出
	int  dequeue(T* node);
	int  dequeue();

	//只看不拿
	int  getNodeAtFront(T* node);
	int  getNodeAtRear(T* node);

	//其他
	bool isEmpty();
	bool isFull();
	int  getCount();
	void clearAll();

private:
	int f;		//front		//f+1為queue的第一個
	int r;		//rear		//r  為queue的最後一個 (最新enqueue者)
	T*	queue;
	int _maxSize;
	pthread_mutex_t m_cs;
};


template <class T>
MyQueue<T>::MyQueue(int maxSize)
{
	if(maxSize <=0) throw std::invalid_argument("MyQueue's size should be >0");

	_maxSize	= maxSize+1;//環狀演算法會浪費一個空間
	f			= 0;
	r			= 0;
	queue		= new T[_maxSize];
	pthread_mutex_init(&m_cs, nullptr);
}

template <class T>
MyQueue<T>::~MyQueue(void)
{
	delete [] queue;
}

template <class T>
int MyQueue<T>::enqueue(T node) //todo: change to T* node?
{
    if(isFull()) return -1; 

	pthread_mutex_lock(&m_cs);
	r = (r+1) % _maxSize; 
	queue[r] = node; 
	pthread_mutex_unlock(&m_cs);
	return 1;
}

template <class T>
int MyQueue<T>::dequeue(T* node)
{
    if(isEmpty()) return -1;

	pthread_mutex_lock(&m_cs);
	f = (f+1) % _maxSize;
	*node = queue[f];
	pthread_mutex_unlock(&m_cs);
    return 1;
}

template <class T>
int MyQueue<T>::dequeue()
{
    if(isEmpty()) return -1;

	pthread_mutex_lock(&m_cs);
    f = (f+1) % _maxSize; 
	pthread_mutex_unlock(&m_cs);
    return 1;
}

template <class T>
int MyQueue<T>::getNodeAtFront(T* node)
{
	if(isEmpty()) {node = NULL; return -1;}

	pthread_mutex_lock(&m_cs);
	*node = queue[(f+1) % _maxSize]; //didn't move f.
	pthread_mutex_unlock(&m_cs);
    return 1;
}

template <class T>
int MyQueue<T>::getNodeAtRear(T* node)
{
    if(isEmpty()) return -1;
	if(_maxSize <=0) 
		return -1;

	pthread_mutex_lock(&m_cs);
	*node = queue[r];
	pthread_mutex_unlock(&m_cs);
    return 1;
}

template <class T>
bool MyQueue<T>::isEmpty()
{
	return r==f;
}

template <class T>
bool MyQueue<T>::isFull()
{
	return (r+1) % _maxSize == f;
}

template <class T>
int  MyQueue<T>::getCount()
{
	int diff = r - f;
	return diff>=0? diff: _maxSize+diff;
}

template <class T>
void  MyQueue<T>::clearAll()
{
	r = f;
}