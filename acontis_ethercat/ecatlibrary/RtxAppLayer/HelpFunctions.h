#pragma once
template <class T>
void CopyArray(T* dst, const T* src, int num)
{
	for(int i = 0; i<num; i++)
		dst[i] = src[i];
}

template <class T>
bool IsArrayEqual(T* dst, const T* src, int num)
{
	for(int i = 0; i<num; i++)
		if(dst[i] != src[i]) 
			return false;

	return true;
}

template <class T>
void SetArrayValue(T* dst, T value, int num)
{
	for(int i = 0; i<num; i++)
		dst[i] = value;
}