#pragma once
#include <iostream>
#include <stdexcept>
using namespace std;

template <class T>
class MyQueueNoLock
{
public:
	MyQueueNoLock(int maxSize);
	~MyQueueNoLock(void);

	//�q���ݥ[�J
	int  enqueue(T node);
	
	//�q�Y���X
	int  dequeue(T* node);
	int  dequeue();

	//�u�ݤ���
	int  getNodeAtFront(T* node);
	int  getNodeAtRear(T* node);

	//��L
	bool isEmpty();
	bool isFull();
	int  getCount();
	void clearAll();

private:
	int f;		//front		//f+1��queue���Ĥ@��
	int r;		//rear		//r  ��queue���̫�@�� (�̷senqueue��)
	T*	queue;
	int _maxSize;
};


template <class T>
MyQueueNoLock<T>::MyQueueNoLock(int maxSize)
{
	if(maxSize <=0) throw std::invalid_argument("MyQueueNoLock's size should be >0");

	_maxSize	= maxSize+1;//�����t��k�|���O�@�ӪŶ�
	f			= 0;
	r			= 0;
	queue		= new T[_maxSize];
}

template <class T>
MyQueueNoLock<T>::~MyQueueNoLock(void)
{
	delete [] queue;
}

template <class T>
int MyQueueNoLock<T>::enqueue(T node) //todo: change to T* node?
{
    if(isFull()) return -1; 


	r = (r+1) % _maxSize; 
	queue[r] = node; 
	
	return 1;
}

template <class T>
int MyQueueNoLock<T>::dequeue(T* node)
{
    if(isEmpty()) return -1;


	f = (f+1) % _maxSize;
	*node = queue[f];
	
    return 1;
}

template <class T>
int MyQueueNoLock<T>::dequeue()
{
    if(isEmpty()) return -1;


    f = (f+1) % _maxSize; 
	
    return 1;
}

template <class T>
int MyQueueNoLock<T>::getNodeAtFront(T* node)
{
	if(isEmpty()) {node = NULL; return -1;}


	*node = queue[(f+1) % _maxSize]; //didn't move f.
	
    return 1;
}

template <class T>
int MyQueueNoLock<T>::getNodeAtRear(T* node)
{
    if(isEmpty()) return -1;
	if(_maxSize <=0) 
		return -1;


	*node = queue[r];
	
    return 1;
}

template <class T>
bool MyQueueNoLock<T>::isEmpty()
{
	return r==f;
}

template <class T>
bool MyQueueNoLock<T>::isFull()
{
	return (r+1) % _maxSize == f;
}

template <class T>
int  MyQueueNoLock<T>::getCount()
{
	int diff = r - f;
	return diff>=0? diff: _maxSize+diff;
}

template <class T>
void  MyQueueNoLock<T>::clearAll()
{
	r = f;
}